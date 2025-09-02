/***********************************************************************************************************************
*                                                                                                                      *
* libscopehal v0.1                                                                                                     *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg and contributors                                                         *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

/*
 * Current State
 * =============
 * - Only basic edge trigger supported. Coupling, hysteresis, B trigger not implemented
 *
 * RS Oscilloscope driver parts (c) 2025 Rick Peironcely, tested on RTM1002
 */


#include "scopehal.h"
#include "RSRTC1002Oscilloscope.h"
#include "EdgeTrigger.h"

#include <cinttypes>

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

RSRTC1002Oscilloscope::RSRTC1002Oscilloscope(SCPITransport* transport)
	: SCPIDevice(transport)
	, SCPIInstrument(transport)
	, m_hasAFG(false)
	, m_triggerArmed(false)
	, m_triggerOneShot(false)
	, m_sampleRateValid(false)
	, m_sampleDepthValid(false)
	, m_triggerOffsetValid(false)
{
	LogDebug("m_model: %s\n", m_model.c_str());
	if (m_model != "RTC1002")
	{
		LogFatal("rs.rtc1002 driver only appropriate for RTC100X");
	}

	SCPISocketTransport* sockettransport = NULL;

	if (!(sockettransport = dynamic_cast<SCPISocketTransport*>(transport)))
	{
		LogFatal("rs.rtc1002 driver requires 'lan' transport");
	}

	//Set up channels
	m_analogChannelCount = 2;
	
	for(unsigned int i=0; i<m_analogChannelCount; i++)
	{
		//Hardware name of the channel
		string chname = string("CHAN1");
		chname[4] += i;

		//Color the channels based on R&S's standard color sequence (yellow-green-orange-bluegray)
		string color = "#ffffff";
		switch(i)
		{
			case 0:
				color = "#ffff00";
				break;

			case 1:
				color = "#00ff00";
				break;

			case 2:
				color = "#ff8000";
				break;

			case 3:
				color = "#8080ff";
				break;
		}

		//Create the channel
		auto chan = new OscilloscopeChannel(
			this,
			chname,
			color,
			Unit(Unit::UNIT_FS),
			Unit(Unit::UNIT_VOLTS),
			Stream::STREAM_TYPE_ANALOG,
			i);
		m_channels.push_back(chan);
		chan->SetDefaultDisplayName();
	}

	// All RTC1002 have external trigger; only edge is supported
	m_extTrigChannel = new OscilloscopeChannel(
		this,
		"EXT",
		"",
		Unit(Unit::UNIT_FS),
		Unit(Unit::UNIT_VOLTS),
		Stream::STREAM_TYPE_TRIGGER,
		m_channels.size());
	m_channels.push_back(m_extTrigChannel);

	m_digitalChannelBase = m_channels.size();
	m_digitalChannelCount = 0;

	string reply = m_transport->SendCommandQueuedWithReply("*OPT?", false);
	vector<string> opts;
	stringstream s_stream(reply);
	while(s_stream.good()) {
		string substr;
		getline(s_stream, substr, ',');
		opts.push_back(substr);
	}

	for (auto app : opts)
	{
		if (app == "B1")
		{
			LogVerbose(" * RTC1002 has logic analyzer/MSO option\n");
			m_digitalChannelCount = 8; // Always 8 channles
		}
		else if (app == "B6")
		{
			LogVerbose(" * RTC1002 has func gen option\n");
			m_hasAFG = true;
		}
		else
		{
			LogDebug("(* Also has option '%s' (ignored))\n", app.c_str());
		}
	}

	// Set up digital channels (if any)
	for(unsigned int i=0; i<m_digitalChannelCount; i++)
	{
		//Hardware name of the channel
		string chname = string("D0");
		chname[1] += i;

		//Create the channel
		auto chan = new OscilloscopeChannel(
			this,
			chname,
			"#555555",
			Unit(Unit::UNIT_FS),
			Unit(Unit::UNIT_COUNTS),
			Stream::STREAM_TYPE_DIGITAL,
			m_channels.size());
		m_channels.push_back(chan);
		chan->SetDefaultDisplayName();
	}

	if (m_digitalChannelCount)
		m_transport->SendCommandQueued("DIG1:THCoupling OFF"); //Allow different threshold per-bank

	if (m_hasAFG)
	{
		//initialize function generator to default settings
		SetFunctionChannelActive(m_firstAFGIndex, false);
		SetFunctionChannelShape(m_firstAFGIndex, FunctionGenerator::SHAPE_SINE);
		SetFunctionChannelFrequency(m_firstAFGIndex, 1.00000000E+03);
		SetFunctionChannelAmplitude(m_firstAFGIndex, 5.0000E-01);
		SetFunctionChannelOffset(m_firstAFGIndex, 0.00E+00);
		SetFunctionChannelDutyCycle(m_firstAFGIndex, 0.25);

		m_firstAFGIndex = m_channels.size();
		auto ch = new FunctionGeneratorChannel(this, "GEN", "#808080", m_channels.size());
		m_channels.push_back(ch);

	}

	m_transport->SendCommandQueued("FORMat:DATA REAL,32"); //Report in f32
	//m_transport->SendCommandQueued("ACQuire:COUNt 1"); //Limit to one acquired waveform per "SINGLE"
	m_transport->SendCommandQueued("EXPort:WAVeform:INCXvalues OFF"); //Don't include X values in data
	m_transport->SendCommandQueued("TIMebase:ROLL:ENABle OFF"); //No roll mode
	m_transport->SendCommandQueued("TRIGGER:A:MODE NORMAL"); //No auto trigger
	//m_transport->SendCommandQueued("ACQuire:CDTA ON"); //All channels have same timebase/etc
	m_transport->SendCommandQueued("PROBe1:SETup:ATTenuation:MANual 10"); //Allow/use manual attenuation setting with unknown probes
	//m_transport->SendCommandQueued("SYSTEM:KLOCK OFF"); //Don't lock front-panel
	m_transport->SendCommandQueued("*WAI");

	GetSampleDepth();

	
}

RSRTC1002Oscilloscope::~RSRTC1002Oscilloscope()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Accessors

string RSRTC1002Oscilloscope::GetDriverNameInternal()
{
	return "rs.rtc1002";
}

unsigned int RSRTC1002Oscilloscope::GetInstrumentTypes() const
{
	unsigned int resp = Instrument::INST_OSCILLOSCOPE;
	if (m_hasAFG)
		resp |= Instrument::INST_FUNCTION;
	return resp;
}

uint32_t RSRTC1002Oscilloscope::GetInstrumentTypesForChannel(size_t i) const
{
	if(m_hasAFG && (i >= m_firstAFGIndex))
		return Instrument::INST_FUNCTION;
	return Instrument::INST_OSCILLOSCOPE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device interface functions

void RSRTC1002Oscilloscope::FlushConfigCache()
{
	LogDebug("FlushConfigCache\n");
	lock_guard<recursive_mutex> lock(m_cacheMutex);

	m_channelOffsets.clear();
	m_channelVoltageRanges.clear();
	m_channelsEnabled.clear();
	m_channelDigitalThresholds.clear();
	m_channelCouplings.clear();
	m_channelAttenuations.clear();

	delete m_trigger;
	m_trigger = NULL;
}

OscilloscopeChannel* RSRTC1002Oscilloscope::GetExternalTrigger()
{
	return m_extTrigChannel;
}

bool RSRTC1002Oscilloscope::IsChannelEnabled(size_t i)
{
	LogDebug("IsChannelEnabled\n");
	if(i == m_extTrigChannel->GetIndex())
		return false;

	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);

		if(m_channelsEnabled.find(i) != m_channelsEnabled.end())
			return m_channelsEnabled[i];
	}

	bool resp;

	if (IsAnalog(i))
	{
		resp = m_transport->SendCommandQueuedWithReply(
							m_channels[i]->GetHwname() + ":STATE?") == "1";
	}
	else
	{
		resp = m_transport->SendCommandQueuedWithReply(
							"LOGic" + to_string(HWDigitalNumber(i)) + ":STATE?") == "ON";
	}

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelsEnabled[i] = resp;
	return m_channelsEnabled[i];
}

void RSRTC1002Oscilloscope::EnableChannel(size_t i)
{
	if(i == m_extTrigChannel->GetIndex())
		return;

	lock_guard<recursive_mutex> lock(m_mutex);

	if (IsAnalog(i))
		m_transport->SendCommandImmediate(m_channels[i]->GetHwname() + ":STATE 1; *WAI");
	else
		m_transport->SendCommandImmediate("BUS1:PAR:BIT" + to_string(HWDigitalNumber(i)) + ":STATE 1; *WAI");

	lock_guard<recursive_mutex> lock2(m_cacheMutex);

	if (IsAnalog(i))
		m_channelsEnabled[i] = true; // Digital channel may fail to enable if pod not connected
}

void RSRTC1002Oscilloscope::DisableChannel(size_t i)
{
	LogDebug("DisableChannel\n");
	if(i == m_extTrigChannel->GetIndex())
		return;

	lock_guard<recursive_mutex> lock(m_mutex);

	if (IsAnalog(i))
		m_transport->SendCommandImmediate(m_channels[i]->GetHwname() + ":STATE 0; *WAI");
	else
		m_transport->SendCommandImmediate("BUS1:PAR:BIT" + to_string(HWDigitalNumber(i)) + ":STATE 0; *WAI");

	lock_guard<recursive_mutex> lock2(m_cacheMutex);
	m_channelsEnabled[i] = false;
}

vector<OscilloscopeChannel::CouplingType> RSRTC1002Oscilloscope::GetAvailableCouplings(size_t i)
{
	LogDebug("GetAvailableCouplings\n");
	vector<OscilloscopeChannel::CouplingType> ret;

	if (IsAnalog(i))
	{
		ret.push_back(OscilloscopeChannel::COUPLE_DC_1M);
		ret.push_back(OscilloscopeChannel::COUPLE_AC_1M);
	}

	ret.push_back(OscilloscopeChannel::COUPLE_DC_50);
	return ret;
}

OscilloscopeChannel::CouplingType RSRTC1002Oscilloscope::GetChannelCoupling(size_t i)
{
	LogDebug("GetChannelCoupling\n");
	if (!IsAnalog(i))
		return OscilloscopeChannel::COUPLE_DC_50;
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelCouplings.find(i) != m_channelCouplings.end())
			return m_channelCouplings[i];
	}
	string reply = m_transport->SendCommandQueuedWithReply(m_channels[i]->GetHwname() + ":COUP?");
	OscilloscopeChannel::CouplingType coupling;
	if(reply == "AC")
		coupling = OscilloscopeChannel::COUPLE_AC_1M;
	else if(reply == "DCL" || reply == "DCLimit")
		coupling = OscilloscopeChannel::COUPLE_DC_1M;
	else if(reply == "DC")
		coupling = OscilloscopeChannel::COUPLE_DC_50;
	else
	{
		LogWarning("invalid coupling value\n");
		coupling = OscilloscopeChannel::COUPLE_DC_50;
	}
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelCouplings[i] = coupling;
	return coupling;
}

void RSRTC1002Oscilloscope::SetChannelCoupling(size_t i, OscilloscopeChannel::CouplingType type)
{
	LogDebug("SetChannelCoupling\n");
	if (!IsAnalog(i)) return;
	{
		// lock_guard<recursive_mutex> lock(m_mutex);
		switch(type)
		{
			case OscilloscopeChannel::COUPLE_DC_50:
				m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":COUP DC");
				break;
			case OscilloscopeChannel::COUPLE_AC_1M:
				m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":COUP AC");
				break;
			case OscilloscopeChannel::COUPLE_DC_1M:
				m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":COUP DCLimit");
				break;
			default:
				LogError("Invalid coupling for channel\n");
		}
	}
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelCouplings[i] = type;
}

// PROBE1:SETUP:ATT:MODE?
//  If MAN: PROBE1:SETUP:GAIN:MANUAL?
//  If AUTO: PROBE1:SETUP:ATT?

double RSRTC1002Oscilloscope::GetChannelAttenuation(size_t i)
{
	LogDebug("GetChannelAttenuation\n");
	if (!IsAnalog(i))
		return 1;
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelAttenuations.find(i) != m_channelAttenuations.end())
			return m_channelAttenuations[i];
	}
	string reply;
	reply = m_transport->SendCommandQueuedWithReply(
						"PROBE" + to_string(i+1) + ":SETUP:ATT:MODE?");
	double attenuation;
	if (reply == "MAN")
	{
		reply = m_transport->SendCommandQueuedWithReply(
						"PROBE" + to_string(i+1) + ":SETUP:GAIN:MANUAL?");
		attenuation = stod(reply);
	}
	else
	{
		reply = m_transport->SendCommandQueuedWithReply(
						"PROBE" + to_string(i+1) + ":SETUP:ATT?");
		attenuation = stod(reply);
	}
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelAttenuations[i] = attenuation;
	return attenuation;
}

void RSRTC1002Oscilloscope::SetChannelAttenuation(size_t i, double atten)
{
	LogDebug("SetChannelAttenuation\n");
	if (!IsAnalog(i)) return;
	string reply;
	reply = m_transport->SendCommandQueuedWithReply(
						"PROBE" + to_string(i+1) + ":SETUP:ATT:MODE?");
	if (reply == "MAN")
	{
		m_transport->SendCommandQueued(
						"PROBE" + to_string(i+1) + ":SETUP:GAIN:MANUAL " + to_string(atten));
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelAttenuations[i] = atten;
	}
	else
	{
		// Can't override attenuation of known probe type
	}
}

std::string RSRTC1002Oscilloscope::GetProbeName(size_t i)
{
	LogDebug("GetProbeName\n");
	if (!IsAnalog(i))
		return "";
	return m_transport->SendCommandQueuedWithReply(
						"PROBE" + to_string(i+1) + ":SETUP:NAME?");
}

unsigned int RSRTC1002Oscilloscope::GetChannelBandwidthLimit(size_t i)
{
	LogDebug("GetChannelBandwidthLimit\n");
	if (!IsAnalog(i))
		return 0;
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelBandwidthLimits.find(i) != m_channelBandwidthLimits.end())
			return m_channelBandwidthLimits[i];
	}
	string reply;
	reply = m_transport->SendCommandQueuedWithReply(m_channels[i]->GetHwname() + ":BANDWIDTH?");
	unsigned int bw = 0;
	if (reply == "FULL")
	{
		bw = 0;
	}
	else if (reply == "B200")
	{
		bw = 200;
	}
	else if (reply == "B20")
	{
		bw = 20;
	}
	else
	{
		LogWarning("Unknown reported bandwidth: %s\n", reply.c_str());
	}
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelBandwidthLimits[i] = bw;
	return bw;
}

void RSRTC1002Oscilloscope::SetChannelBandwidthLimit(size_t i, unsigned int limit_mhz)
{
	LogDebug("SetChannelBandwidthLimit\n");
	if (!IsAnalog(i)) return;
	LogDebug("Request bandwidth: %u\n", limit_mhz);
	string limit_str;
	if (limit_mhz == 0)
	{
		limit_str = "FULL";
		limit_mhz = 0;
	}
	else if (limit_mhz == 20)
	{
		limit_str = "B20";
		limit_mhz = 20;
	}
	else if (limit_mhz == 200)
	{
		limit_str = "B200";
		limit_mhz = 200;
	}
	else
	{
		LogWarning("Unsupported requested bandwidth\n");
		return;
	}
	m_transport->SendCommandQueued(m_channels[i]->GetHwname() + ":BANDWIDTH " + limit_str);
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelBandwidthLimits[i] = limit_mhz;
}

vector<unsigned int> RSRTC1002Oscilloscope::GetChannelBandwidthLimiters(size_t i)
{
	LogDebug("GetChannelBandwidthLimiters\n");
	vector<unsigned int> ret;

	if (IsAnalog(i))
	{
		ret.push_back(20);
		ret.push_back(200);
	}

	ret.push_back(0);
	return ret;
}

float RSRTC1002Oscilloscope::GetChannelVoltageRange(size_t i, size_t /*stream*/)
{
	LogDebug("GetChannelVoltageRange\n");
	if (!IsAnalog(i)) return 0;

	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		if(m_channelVoltageRanges.find(i) != m_channelVoltageRanges.end())
			return m_channelVoltageRanges[i];
	}

	string reply = m_transport->SendCommandQueuedWithReply(m_channels[i]->GetHwname() + ":RANGE?");

	float range;
	sscanf(reply.c_str(), "%f", &range);
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelVoltageRanges[i] = range;
	return range;
}

void RSRTC1002Oscilloscope::SetChannelVoltageRange(size_t i, size_t /*stream*/, float range)
{
	LogDebug("SetChannelVoltageRange\n");
	if (!IsAnalog(i)) return;

	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelVoltageRanges[i] = range;
	}

	char cmd[128];
	snprintf(cmd, sizeof(cmd), "%s:RANGE %.4f", m_channels[i]->GetHwname().c_str(), range);
	m_transport->SendCommandQueued(cmd);
}

float RSRTC1002Oscilloscope::GetChannelOffset(size_t i, size_t /*stream*/)
{
	LogDebug("GetChannelOffset\n");
	if (!IsAnalog(i)) return 0;

	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);

		if(m_channelOffsets.find(i) != m_channelOffsets.end())
			return m_channelOffsets[i];
	}

	// lock_guard<recursive_mutex> lock2(m_mutex);

	string reply = m_transport->SendCommandQueuedWithReply(m_channels[i]->GetHwname() + ":OFFS?");

	float offset;
	sscanf(reply.c_str(), "%f", &offset);
	offset = -offset;
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelOffsets[i] = offset;
	return offset;
}

void RSRTC1002Oscilloscope::SetChannelOffset(size_t i, size_t /*stream*/, float offset)
{
	LogDebug("SetChannelOffset\n");
	if (!IsAnalog(i)) return;

	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelOffsets[i] = offset;
	}

	// lock_guard<recursive_mutex> lock(m_mutex);
	char cmd[128];
	snprintf(cmd, sizeof(cmd), "%s:OFFS %.4f", m_channels[i]->GetHwname().c_str(), -offset);
	m_transport->SendCommandQueued(cmd);
}

//////////////////////////////////////////////////////////////////////////////// <Digital>

vector<Oscilloscope::DigitalBank> RSRTC1002Oscilloscope::GetDigitalBanks()
{
	LogDebug("GetDigitalBanks\n");
	vector<DigitalBank> banks;

	for (unsigned int i = 0; i < m_digitalChannelCount; i += 4)
	{
		DigitalBank bank;
		for (int n = 0; n < 4; n++)
			bank.push_back(static_cast<OscilloscopeChannel*>(m_channels[m_digitalChannelBase + i + n]));
		banks.push_back(bank);
	}

	return banks;
}

Oscilloscope::DigitalBank RSRTC1002Oscilloscope::GetDigitalBank(size_t channel)
{
	LogDebug("GetDigitalBank\n");
	return GetDigitalBanks()[HWDigitalNumber(channel) - (HWDigitalNumber(channel) % 4)];
}

bool RSRTC1002Oscilloscope::IsDigitalHysteresisConfigurable()
{
	return false;
	// TODO: It is "sorta" configurable... but not as a settable value
	// See https://www.rohde-schwarz.com/webhelp/RTC1002_HTML_UserManual_en/Content/5af36d65427b4b68.htm
}

bool RSRTC1002Oscilloscope::IsDigitalThresholdConfigurable()
{
	return true;
}

float RSRTC1002Oscilloscope::GetDigitalThreshold(size_t channel)
{
	LogDebug("GetDigitalThreshold\n");
	if( (channel < m_digitalChannelBase) || (m_digitalChannelCount == 0) )
		return 0;

	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);

		if(m_channelDigitalThresholds.find(channel) != m_channelDigitalThresholds.end())
			return m_channelDigitalThresholds[channel];
	}

	float result = stof(m_transport->SendCommandQueuedWithReply("DIG" + to_string(HWDigitalNumber(channel)) + ":THR?"));

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	m_channelDigitalThresholds[channel] = result;
	return result;
}

void RSRTC1002Oscilloscope::SetDigitalThreshold(size_t channel, float level)
{
	LogDebug("SetDigitalThreshold\n");
	m_transport->SendCommandQueuedWithReply("DIG" + to_string(HWDigitalNumber(channel)) + ":THR " + to_string(level));
}

//////////////////////////////////////////////////////////////////////////////// </Digital>

Oscilloscope::TriggerMode RSRTC1002Oscilloscope::PollTrigger()
{
	LogDebug("PollTrigger\n");
	// lock_guard<recursive_mutex> lock(m_mutex);
	if (!m_triggerArmed)
		return TRIGGER_MODE_STOP;

	////////////////////////////////////////////////////////////////////////////
	string state = m_transport->SendCommandQueuedWithReply("ACQuire:STATe?");

	if (state == "RUN")
	{
		return TRIGGER_MODE_RUN;
	}
	else
	{
		if ((state != "COMPlete") && (state != "STOP"))
			LogWarning("ACQuire:CURRent? -> %s\n", state.c_str());

		m_triggerArmed = false;
		return TRIGGER_MODE_TRIGGERED;
	}

	// return m_triggerArmed ? TRIGGER_MODE_TRIGGERED : TRIGGER_MODE_STOP;
}

template <typename T> size_t RSRTC1002Oscilloscope::AcquireHeader(T* cap, string chname)
{
	//LogDebug("AcquireHeader\n");
	//This is basically the same function as a LeCroy WAVEDESC, but much less detailed
	string reply = m_transport->SendCommandImmediateWithReply(chname + ":DATA:HEAD?; *WAI");

	double xstart;
	double xstop;
	size_t length;
	int samples_per_interval;
	int rc = sscanf(reply.c_str(), "%lf,%lf,%zu,%d", &xstart, &xstop, &length, &samples_per_interval);
	if (samples_per_interval != 1)
		LogFatal("Don't understand samples_per_interval != 1");

	if (rc != 4 || length == 0)
	{
		/* No data - Skip query the scope and move on */
		return 0;
	}

	//Figure out the sample rate
	double capture_len_sec = xstop - xstart;
	double sec_per_sample = capture_len_sec / length;
	int64_t fs_per_sample = round(sec_per_sample * FS_PER_SECOND);
	LogDebug("%" PRId64 " fs/sample\n", fs_per_sample);

	size_t reported_srate = (FS_PER_SECOND / fs_per_sample);

	if (reported_srate != m_sampleRate)
	{
		LogWarning("Reported sample rate %zu != expected sample rate %" PRIu64 "; using what it said\n", reported_srate, m_sampleRate);
	}

	if (length != m_sampleDepth)
	{
		LogWarning("Reported depth %zu != expected depth %" PRIu64 "; using what I think is correct\n", length, m_sampleDepth);
		length = m_sampleDepth;
	}

	//Set up the capture we're going to store our data into (no high res timer on R&S scopes)

	cap->m_timescale = fs_per_sample;
	cap->m_triggerPhase = 0;
	cap->m_startTimestamp = time(NULL);
	double t = GetTime();
	cap->m_startFemtoseconds = (t - floor(t)) * FS_PER_SECOND;

	cap->Resize(length);
	cap->PrepareForCpuAccess();

	return length;
}

bool RSRTC1002Oscilloscope::AcquireData()
{
	LogDebug(" ** AcquireData ** \n");
	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->FlushCommandQueue();
	LogIndenter li;

	GetSampleDepth();

	auto start_time = std::chrono::system_clock::now();

	// m_transport->SendCommandQueued("*DCL; *WAI");

	map<int, vector<WaveformBase*> > pending_waveforms;
	bool any_data = false;

	for(size_t i=0; i<m_analogChannelCount; i++)
	{
		if(!IsChannelEnabled(i))
			continue;

		LogDebug("Starting acquisition phase for ch%zu\n", i);

		auto cap = new UniformAnalogWaveform;
		size_t length = AcquireHeader(cap, m_channels[i]->GetHwname());

		if (!length)
		{
			delete cap;
			pending_waveforms[i].push_back(NULL);
			continue;
		}

		any_data = true;

		size_t transferred = 0;
		// Request a reasonably-sized buffer as this may cause RAM allocation in recv(2)
		const size_t block_size = 50e6;

		unsigned char* dest_buf = (unsigned char*)cap->m_samples.GetCpuPointer();

		LogDebug(" - Begin transfer of %zu bytes\n", length);

		while (transferred != length)
		{
			size_t this_length = block_size;
			if (this_length > (length - transferred))
				this_length = length - transferred;

			string params =  " "+to_string(transferred)+","+to_string(this_length);

			if (transferred == 0 && this_length == length)
				params = "";

			LogDebug("[%3d%%] Query ...`DATA?%s` (B)\n", (int)(100*((float)transferred/(float)length)), params.c_str());

			//Ask for the data
			size_t len_bytes;
			unsigned char* samples = (unsigned char*)m_transport->SendCommandImmediateWithRawBlockReply(m_channels[i]->GetHwname() + ":DATA?"+params+"; *WAI", len_bytes);

			if (len_bytes != (this_length*sizeof(float)))
			{
				LogError("Unexpected number of bytes back; aborting acquisition");
				std::this_thread::sleep_for(std::chrono::microseconds(100200));
				m_transport->FlushRXBuffer();

				delete cap;

				for (auto* c : pending_waveforms[i])
				{
					delete c;
				}

				delete[] samples;

				return false;
			}

			unsigned char* cpy_target = dest_buf+(transferred*sizeof(float));
			// LogDebug("Copying %zuB from %p to %p\n", len_bytes, samples, cpy_target);

			memcpy(cpy_target, samples, len_bytes);
			transferred += this_length;
			delete[] samples;

			//Discard trailing newline
			uint8_t disregard;
			m_transport->ReadRawData(1, &disregard);
		}

		LogDebug("[100%%] Done\n");

		cap->MarkSamplesModifiedFromCpu();

		//Done, update the data
		pending_waveforms[i].push_back(cap);
	}

	bool didAcquireAnyDigitalChannels = false;

	for(size_t i=m_digitalChannelBase; i<(m_digitalChannelBase + m_digitalChannelCount); i++)
	{
		if(!IsChannelEnabled(i))
			continue;

		if (!didAcquireAnyDigitalChannels)
		{
			while (m_transport->SendCommandImmediateWithReply("FORM?") != "ASC,0")
			{
				m_transport->SendCommandImmediate("FORM ASC; *WAI"); //Only possible to get data out in ASCII format
				std::this_thread::sleep_for(std::chrono::microseconds(1002000));
			}
			didAcquireAnyDigitalChannels = true;
		}

		string hwname = "DIG" + to_string(HWDigitalNumber(i));

		LogDebug("Starting acquisition for dig%d\n", HWDigitalNumber(i));

		auto cap = new SparseDigitalWaveform;
		size_t length = AcquireHeader(cap, hwname);

		if (!length)
		{
			delete cap;
			pending_waveforms[i].push_back(NULL);
			continue;
		}

		size_t expected_bytes = length * 2; // Commas between items + newline

		// Digital channels do not appear to support selecting a subset, so no 'chunking'

		LogDebug(" - Begin transfer of %zu bytes (*2)\n", length);

		// Since it's ascii the scope just sends it as a SCPI 'line' without the size block
		m_transport->SendCommandImmediate(hwname + ":DATA?; *WAI");
		unsigned char* samples = new unsigned char[expected_bytes];
		size_t read_bytes = m_transport->ReadRawData(expected_bytes, samples);

		if (read_bytes != expected_bytes)
		{
			LogWarning("Unexpected number of bytes back; aborting acquisiton\n");
			std::this_thread::sleep_for(std::chrono::microseconds(100200));
			m_transport->FlushRXBuffer();

			delete cap;

			for (auto* c : pending_waveforms[i])
			{
				delete c;
			}

			delete[] samples;

			return false;
		}

		bool last = samples[0] == '1';

		cap->m_offsets[0] = 0;
		cap->m_durations[0] = 1;
		cap->m_samples[0] = last;

		size_t k = 0;

		for(size_t m=1; m<length; m++)
		{
			bool sample = samples[m*2] == '1';

			//Deduplicate consecutive samples with same value
			//FIXME: temporary workaround for rendering bugs
			//if(last == sample)
			if( (last == sample) && ((m+5) < length) && (m > 5))
				cap->m_durations[k] ++;

			//Nope, it toggled - store the new value
			else
			{
				k++;
				cap->m_offsets[k] = m;
				cap->m_durations[k] = 1;
				cap->m_samples[k] = sample;
				last = sample;
			}
		}

		//Free space reclaimed by deduplication
		cap->Resize(k);
		cap->m_offsets.shrink_to_fit();
		cap->m_durations.shrink_to_fit();
		cap->m_samples.shrink_to_fit();

		cap->MarkSamplesModifiedFromCpu();
		cap->MarkTimestampsModifiedFromCpu();

		delete[] samples;

		//Done, update the data
		pending_waveforms[i].push_back(cap);
	}

	if (didAcquireAnyDigitalChannels)
		m_transport->SendCommandImmediate("FORMat:DATA REAL,32"); //Return to f32

	if (any_data)
	{
		//Now that we have all of the pending waveforms, save them in sets across all channels
		m_pendingWaveformsMutex.lock();
		size_t num_pending = 1;	//TODO: segmented capture support
		for(size_t i=0; i<num_pending; i++)
		{
			SequenceSet s;
			for(size_t j=0; j<m_channels.size(); j++)
			{
				if(IsChannelEnabled(j))
					s[m_channels[j]] = pending_waveforms[j][i];
			}
			m_pendingWaveforms.push_back(s);
		}
		m_pendingWaveformsMutex.unlock();
	}

	if(!any_data || !m_triggerOneShot)
	{
		m_transport->SendCommandImmediate("SINGle");
		std::this_thread::sleep_for(std::chrono::microseconds(100200));
		// If we don't wait here, sending the query for available waveforms will race and return 1 for the exitisting waveform and jam everything up.
		m_triggerArmed = true;
	}
	else
	{
		m_triggerArmed = false;
	}

	auto end_time = std::chrono::system_clock::now();

	LogDebug("Acquisition took %" PRId64 "\n", std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count());

	return any_data;
}

void RSRTC1002Oscilloscope::Start()
{
	LogDebug("Start\n");
	m_transport->SendCommandImmediate("SINGle");
	std::this_thread::sleep_for(std::chrono::microseconds(100000));
	// If we don't wait here, sending the query for available waveforms will race and return 1 for the exitisting waveform and jam everything up.
	m_triggerArmed = true;
	m_triggerOneShot = false;
}

void RSRTC1002Oscilloscope::StartSingleTrigger()
{
	LogDebug("StartSingleTrigger\n");
	m_transport->SendCommandImmediate("SINGle");
	std::this_thread::sleep_for(std::chrono::microseconds(100000));
	// If we don't wait here, sending the query for available waveforms will race and return 1 for the exitisting waveform and jam everything up.
	m_triggerArmed = true;
	m_triggerOneShot = true;
}

void RSRTC1002Oscilloscope::Stop()
{
	LogDebug("Stop\n");
	m_triggerArmed = false;

	LogDebug("Stop!\n");
	m_transport->SendCommandImmediate("STOP");
	m_triggerArmed = false;
	m_triggerOneShot = true;
}

void RSRTC1002Oscilloscope::ForceTrigger()
{
	LogDebug("ForceTrigger\n");
	if (m_triggerArmed)
		m_transport->SendCommandImmediate("TRG");
}

bool RSRTC1002Oscilloscope::IsTriggerArmed()
{
	return m_triggerArmed;
}

vector<uint64_t> RSRTC1002Oscilloscope::GetSampleRatesNonInterleaved()
{
	LogDebug("GetSampleRatesNonInterleaved\n");
	LogWarning("RSRTC1002Oscilloscope::GetSampleRatesNonInterleaved unimplemented\n");

	// FIXME -- Arbitrarily copied from Tek
	vector<uint64_t> ret;

	const int64_t k = 1002;
	const int64_t m = k*k;
	const int64_t g = k*m;

	uint64_t bases[] = { 1002, 1250, 2500, 3125, 5000, 6250 };
	vector<uint64_t> scales = {1, 10, 100, 1*k};

	for(auto b : bases)
		ret.push_back(b / 10);

	for(auto scale : scales)
	{
		for(auto b : bases)
			ret.push_back(b * scale);
	}

	// // MSO6 also supports these, or at least had them available in the picker before.
	// // TODO: Are these actually supported?

	// if (m_family == FAMILY_MSO6) {
	// 	for(auto b : bases) {
	// 		ret.push_back(b * 10 * k);
	// 	}
	// }

	// We break with the pattern on the upper end of the frequency range
	ret.push_back(12500 * k);
	ret.push_back(25 * m);
	ret.push_back(31250 * k);
	ret.push_back(62500 * k);
	ret.push_back(125 * m);
	ret.push_back(250 * m);
	ret.push_back(312500 * k);
	ret.push_back(625 * m);
	ret.push_back(1250 * m);
	ret.push_back(1562500 * k);
	ret.push_back(3125 * m);
	ret.push_back(6250 * m);
	ret.push_back(12500 * m);

	// Below are interpolated. 8 bits, not 12.
	//TODO: we can save bandwidth by using 8 bit waveform download for these

	ret.push_back(25 * g);

	// MSO5 supports these, TODO: Does MSO6?
	ret.push_back(25000 * m);
	ret.push_back(62500 * m);
	ret.push_back(125000 * m);
	ret.push_back(250000 * m);
	ret.push_back(500000 * m);

	return ret;
}

vector<uint64_t> RSRTC1002Oscilloscope::GetSampleRatesInterleaved()
{
	return GetSampleRatesNonInterleaved();
}

set<Oscilloscope::InterleaveConflict> RSRTC1002Oscilloscope::GetInterleaveConflicts()
{
	LogDebug("GetInterleaveConflicts\n");
	LogWarning("RSRTC1002Oscilloscope::GetInterleaveConflicts unimplemented\n");

	//FIXME
	set<Oscilloscope::InterleaveConflict> ret;
	return ret;
}

vector<uint64_t> RSRTC1002Oscilloscope::GetSampleDepthsNonInterleaved()
{
	LogDebug("GetSampleDepthsNonInterleaved\n");
	LogWarning("RSRTC1002Oscilloscope::GetSampleDepthsNonInterleaved unimplemented\n");

	//FIXME -- Arbitrarily copied from Tek
	vector<uint64_t> ret;

	const int64_t k = 1002;
	const int64_t m = k*k;
	// const int64_t g = k*m;

	ret.push_back(500);
	ret.push_back(1 * k);
	ret.push_back(2 * k);
	ret.push_back(5 * k);
	ret.push_back(10 * k);
	ret.push_back(20 * k);
	ret.push_back(50 * k);
	ret.push_back(100 * k);
	ret.push_back(200 * k);
	ret.push_back(500 * k);

	ret.push_back(1 * m);
	ret.push_back(2 * m);
	ret.push_back(5 * m);
	ret.push_back(10 * m);
	ret.push_back(20 * m);
	ret.push_back(50 * m);
	ret.push_back(62500 * k);
	ret.push_back(100 * m);
	ret.push_back(400 * m);
	ret.push_back(800 * m);

	return ret;
}

vector<uint64_t> RSRTC1002Oscilloscope::GetSampleDepthsInterleaved()
{
	return GetSampleRatesNonInterleaved();
}

uint64_t RSRTC1002Oscilloscope::GetSampleRate()
{
	LogDebug("GetSampleRate\n");
	if(m_sampleRateValid)
	{
		LogDebug("GetSampleRate() queried and returned cached value %" PRIu64 "\n", m_sampleRate);
		return m_sampleRate;
	}

	m_sampleRate = stod(m_transport->SendCommandQueuedWithReply("ACQuire:SRATe?"));
	m_sampleRateValid = true;

	LogDebug("GetSampleRate() queried and got new value %" PRIu64 "\n", m_sampleRate);

	return 1;
}

uint64_t RSRTC1002Oscilloscope::GetSampleDepth()
{
	LogDebug("GetSampleDepth\n");
	if(m_sampleDepthValid)
	{
		LogDebug("GetSampleDepth() queried and returned cached value %" PRIu64 "\n", m_sampleDepth);
		return m_sampleDepth;
	}

	GetSampleRate();

	m_sampleDepth = stod(m_transport->SendCommandQueuedWithReply("TIMEBASE:RANGE?")) * (double)m_sampleRate;
	m_sampleDepthValid = true;

	LogDebug("GetSampleDepth() queried and got new value %" PRIu64 "\n", m_sampleDepth);

	return 1;
}

void RSRTC1002Oscilloscope::SetSampleDepth(uint64_t depth)
{
	LogDebug("SetSampleDepth\n");
	GetSampleRate();

	//Update the cache
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_sampleDepth = depth;
		m_sampleDepthValid = true;
	}

	LogDebug("SetSampleDepth() setting to %" PRIu64 "\n", depth);

	m_transport->SendCommandQueued(string("TIMEBASE:RANGE ") + to_string((double)depth / (double)m_sampleRate));
}

void RSRTC1002Oscilloscope::SetSampleRate(uint64_t rate)
{
	LogDebug("SetSampleRate\n");
	//Update the cache
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_sampleRate = rate;
		m_sampleRateValid = true;
	}

	LogDebug("SetSampleRate() setting to %" PRIu64 "\n", rate);

	m_transport->SendCommandQueued(string("ACQUIRE:SRATE ") + to_string(rate));

	SetSampleDepth(m_sampleDepth);
}

void RSRTC1002Oscilloscope::SetTriggerOffset(int64_t offset)
{
	LogDebug("SetTriggerOffset\n");
	//Update the cache
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_triggerOffset = offset;
		m_triggerOffsetValid = false; // Probably will be rounded and/or clipped
	}

	m_transport->SendCommandQueued("TIMEBASE:HORIZONTAL:POSITION " + to_string(-((double)offset)*((double)SECONDS_PER_FS)));
}

int64_t RSRTC1002Oscilloscope::GetTriggerOffset()
{
	LogDebug("GetTriggerOffset\n");
	if(m_triggerOffsetValid)
	{
		// LogDebug("GetTriggerOffset() queried and returned cached value %ld\n", m_triggerOffset);
		return m_triggerOffset;
	}

	string reply = m_transport->SendCommandQueuedWithReply("TIMEBASE:HORIZONTAL:POSITION?");

	m_triggerOffset = -stof(reply)*FS_PER_SECOND;
	m_triggerOffsetValid = true;

	return m_triggerOffset;
}

bool RSRTC1002Oscilloscope::IsInterleaving()
{
	return false;
}

bool RSRTC1002Oscilloscope::SetInterleaving(bool /*combine*/)
{
	return false;
}

void RSRTC1002Oscilloscope::PullTrigger()
{
	LogDebug("PullTrigger\n");
	lock_guard<recursive_mutex> lock(m_mutex);

	string resp = m_transport->SendCommandQueuedWithReply("TRIGGER:A:TYPE?");

	if (resp == "EDGE")
		PullEdgeTrigger();
	else
	{
		LogWarning("Unknown Trigger Type. Forcing Edge.\n");

		delete m_trigger;

		m_trigger = new EdgeTrigger(this);
		EdgeTrigger* et = dynamic_cast<EdgeTrigger*>(m_trigger);

		et->SetType(EdgeTrigger::EDGE_RISING);
		et->SetInput(0, StreamDescriptor(GetChannelByHwName("CHAN1"), 0), true);
		et->SetLevel(1.0);
		PushTrigger();
		PullTrigger();
	}
}

/**
	@brief Reads settings for an edge trigger from the instrument
 */
void RSRTC1002Oscilloscope::PullEdgeTrigger()
{
	LogDebug("PullEdgeTrigger\n");
	if( (m_trigger != NULL) && (dynamic_cast<EdgeTrigger*>(m_trigger) != NULL) )
	{
		delete m_trigger;
		m_trigger = NULL;
	}

	//Create a new trigger if necessary
	if(m_trigger == NULL)
		m_trigger = new EdgeTrigger(this);
	EdgeTrigger* et = dynamic_cast<EdgeTrigger*>(m_trigger);

	string reply = m_transport->SendCommandQueuedWithReply("TRIGGER:A:SOURCE?");
	et->SetInput(0, StreamDescriptor(GetChannelByHwName(reply), 0), true);

	reply = m_transport->SendCommandQueuedWithReply("TRIGGER:A:EDGE:SLOPE?");
	if (reply == "POS")
		et->SetType(EdgeTrigger::EDGE_RISING);
	else if (reply == "NEG")
		et->SetType(EdgeTrigger::EDGE_FALLING);
	else if (reply == "EITH")
		et->SetType(EdgeTrigger::EDGE_ANY);
	else
	{
		LogWarning("Unknown edge type\n");
		et->SetType(EdgeTrigger::EDGE_ANY);
	}

	reply = m_transport->SendCommandQueuedWithReply("TRIGGER:A:LEVEL?");
	et->SetLevel(stof(reply));
}

void RSRTC1002Oscilloscope::PushTrigger()
{
	LogDebug("PushTrigger\n");
	auto et = dynamic_cast<EdgeTrigger*>(m_trigger);
	if(et)
		PushEdgeTrigger(et);
	else
		LogWarning("Unknown trigger type (not an edge)\n");
}

/**
	@brief Pushes settings for an edge trigger to the instrument
 */
void RSRTC1002Oscilloscope::PushEdgeTrigger(EdgeTrigger* trig)
{
	LogDebug("PushEdgeTrigger\n");

	m_transport->SendCommandQueued("TRIGGER:A:EVENT SINGLE");
	m_transport->SendCommandQueued("TRIGGER:A:TYPE EDGE");
	m_transport->SendCommandQueued(string("TRIGGER:A:SOURCE ") + trig->GetInput(0).m_channel->GetHwname());

	switch(trig->GetType())
	{
		case EdgeTrigger::EDGE_RISING:
			m_transport->SendCommandQueued("TRIGGER:A:EDGE:SLOPE POSITIVE");
			break;

		case EdgeTrigger::EDGE_FALLING:
			m_transport->SendCommandQueued("TRIGGER:A:EDGE:SLOPE NEGATIVE");
			break;

		case EdgeTrigger::EDGE_ANY:
			m_transport->SendCommandQueued("TRIGGER:A:EDGE:SLOPE EITHER");
			break;

		default:
			LogWarning("Unknown edge type\n");
			break;
	}

	m_transport->SendCommandQueued(string("TRIGGER:A:LEVEL") /*+ to_string(trig->GetInput(0).m_channel->GetIndex())*/ + " " + to_string(trig->GetLevel()));
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function generator

vector<FunctionGenerator::WaveShape> RSRTC1002Oscilloscope::GetAvailableWaveformShapes(int /*chan*/)
{
	LogDebug("GetAvailableWaveformShapes\n");
	vector<WaveShape> res;
	for (const auto& i : m_waveShapeNames)
		res.push_back(i.second);

	return res;
}

#define TO_HW_STR(chan) to_string(chan - m_firstAFGIndex + 1)

//Configuration
bool RSRTC1002Oscilloscope::GetFunctionChannelActive(int chan)
{
	LogDebug("GetFunctionChannelActive returning data from cache\n");
	return m_functionActive;
}

void RSRTC1002Oscilloscope::SetFunctionChannelActive(int chan, bool on)
{
	LogDebug("SetFunctionChannelActive\n");
	m_functionActive = on;
	if (on)
		m_transport->SendCommandQueued("GENerator:OUTPut:ENABLE ON");
	else
		m_transport->SendCommandQueued("GENerator:OUTPut:ENABLE OFF");
}

bool RSRTC1002Oscilloscope::HasFunctionDutyCycleControls(int chan)
{
	LogDebug("HasFunctionDutyCycleControls\n");
	return GetFunctionChannelShape(chan) == FunctionGenerator::SHAPE_SQUARE;
}

float RSRTC1002Oscilloscope::GetFunctionChannelDutyCycle(int chan)
{
	LogDebug("GetFunctionChannelDutyCycle returning data from cache\n");
	return m_functionDutyCycle;
}

void RSRTC1002Oscilloscope::SetFunctionChannelDutyCycle(int chan, float duty)
{
	LogDebug("SetFunctionChannelDutyCycle\n");
	if ((duty*100 < 1.000E+01) || (duty*100 > 9.000E+01))
	{
		LogWarning("Invalid duty cycle range\n");
		return;
	}
	m_functionDutyCycle = duty;
	m_transport->SendCommandQueued("GENerator:FUNCtion:PULSe:DCYCle " + to_string(duty*100.));
}

float RSRTC1002Oscilloscope::GetFunctionChannelAmplitude(int chan)
{
	LogDebug("GetFunctionChannelAmplitude returning data from cache\n");
	return m_functionAmplitude;
}

void RSRTC1002Oscilloscope::SetFunctionChannelAmplitude(int chan, float amplitude)
{
	LogDebug("SetFunctionChannelAmplitude\n");
	if ((amplitude < 6.0000E-02) || (amplitude > 6.0000E+01))
	{
		LogWarning("Invalid amplitude range\n");
		return;
	}
	m_functionAmplitude = amplitude;
	m_transport->SendCommandQueued("GENerator:VOLTage " + to_string(amplitude));
	
}

float RSRTC1002Oscilloscope::GetFunctionChannelOffset(int chan)
{
	LogDebug("GetFunctionChannelOffset returning data from cache\n");
	return m_functionOffset;
}

void RSRTC1002Oscilloscope::SetFunctionChannelOffset(int chan, float offset)
{

	LogDebug("SetFunctionChannelOffset\n");
	if (offset < -3.0000E+01 || offset > 3.0000E+01)
	{
		LogWarning("Invalid offset range\n");
		return;
	}
	m_functionOffset = offset;
	m_transport->SendCommandQueued("GENerator:VOLTage:OFFSet " + to_string(offset));
}

float RSRTC1002Oscilloscope::GetFunctionChannelFrequency(int chan)
{
	LogDebug("GetFunctionChannelFrequency returning data from cache\n");
	return m_functionFrequency;
}

void RSRTC1002Oscilloscope::SetFunctionChannelFrequency(int chan, float hz)
{
	LogDebug("SetFunctionChannelFrequency\n");
	float maxFreq = (m_functionShape == FunctionGenerator::SHAPE_TRIANGLE) ?  1.000000000E+04 :  5.000000000E+04;
	if ((hz < 1.0000E-03) || (hz > maxFreq))
	{
		LogWarning("Invalid frequency range\n");
		return;
	}
	m_functionFrequency = hz;
	m_transport->SendCommandQueued("GENerator:FREQuency " + to_string(hz));
}

FunctionGenerator::WaveShape RSRTC1002Oscilloscope::GetFunctionChannelShape(int chan)
{
	LogDebug("GetFunctionChannelShape returning data from cache\n");
	return m_functionShape;
}

void RSRTC1002Oscilloscope::SetFunctionChannelShape(int chan, FunctionGenerator::WaveShape shape)
{
	LogDebug("SetFunctionChannelShape\n");
	for (const auto& i : m_waveShapeNames)
	{
		if (i.second == shape)
		{
			m_functionShape = shape;
			m_transport->SendCommandQueued("GENerator:FUNCtion " + i.first);
			return;
		}
	}

	LogWarning("Unsupported WaveShape requested\n");
}

bool RSRTC1002Oscilloscope::HasFunctionRiseFallTimeControls(int /*chan*/)
{
	return false;
}

FunctionGenerator::OutputImpedance RSRTC1002Oscilloscope::GetFunctionChannelOutputImpedance(int chan)
{
	LogDebug("GetFunctionChannelOutputImpedance\n");
	return FunctionGenerator::IMPEDANCE_50_OHM;
}
void RSRTC1002Oscilloscope::SetFunctionChannelOutputImpedance(int chan, FunctionGenerator::OutputImpedance z)
{
	LogDebug("SetFunctionChannelOutputImpedance not implemented\n");
}

const std::map<const std::string, const FunctionGenerator::WaveShape> RSRTC1002Oscilloscope::m_waveShapeNames = {
		{"SIN", FunctionGenerator::SHAPE_SINE},
		{"SQU", FunctionGenerator::SHAPE_SQUARE},
		{"RAMP", FunctionGenerator::SHAPE_TRIANGLE},
		{"DC", FunctionGenerator::SHAPE_DC},
		{"PULS", FunctionGenerator::SHAPE_PULSE}
		// {"SINC", FunctionGenerator::SHAPE_SINC}, // Not supported
		// {"CARD", FunctionGenerator::SHAPE_CARDIAC}, // Not supported
		// {"GAUS", FunctionGenerator::SHAPE_GAUSSIAN}, // Not supported
		// {"LORN", FunctionGenerator::SHAPE_LORENTZ}, // Not supported
		// {"EXPR", FunctionGenerator::SHAPE_EXPONENTIAL_RISE}, // Not supported
		// {"EXPF", FunctionGenerator::SHAPE_EXPONENTIAL_DECAY}, // Not supported
		// {"", FunctionGenerator::SHAPE_ARB} // Not supported
};

bool RSRTC1002Oscilloscope::HasErrors()
{
	LogDebug("HasErrors\n");
	string reply = m_transport->SendCommandQueuedWithReply("STB?");
	//is bit2 set to 1?
	if (reply.size() > 1 && reply[1] == '1')
		return true;
	return false;
}

bool RSRTC1002Oscilloscope::ReadErrors()
{
	LogDebug("ReadErrors\n");
	if (!HasErrors())
	{
		return false;
	}
	while (true)
	{
		string reply = m_transport->SendCommandQueuedWithReply("SYSTem:ERRor?");
		if (reply == '0,\”No Error\”')
		{
			break
		}
		LogError("Unknown error response: " + reply);
	}
	return true;
}