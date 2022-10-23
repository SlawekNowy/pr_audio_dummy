/* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "pr_audio_dummy.hpp"
#include <alsoundsystem.hpp>
#include <alsound_source.hpp>
#include <alsound_buffer.hpp>
#include <alsound_listener.hpp>
#include <sharedutils/util_pragma.hpp>


#ifdef __linux__
#define DLLEXPORT __attribute__((visibility("default")))
#else
#define DLLEXPORT __declspec(dllexport)
#endif
namespace al
{
	class DummySoundSystem
		: public ISoundSystem
	{
	public:
		DummySoundSystem(float metersPerUnit);
		virtual PEffect CreateEffect() {return nullptr;}
		virtual IAuxiliaryEffectSlot *CreateAuxiliaryEffectSlot() override {return nullptr;}
		
		virtual PDecoder CreateDecoder(const std::string &path,bool bConvertToMono=false) override {return nullptr;}
		virtual bool IsSupported(ChannelConfig channels,SampleType type) const override {return true;}

		virtual float GetDopplerFactor() const override {return m_dopplerFactor;}
		virtual void SetDopplerFactor(float factor) override {m_dopplerFactor = factor;}

		virtual float GetSpeedOfSound() const override {return m_speedOfSound;}
		virtual void SetSpeedOfSound(float speed) override {m_speedOfSound = speed;}

		virtual DistanceModel GetDistanceModel() const override {return m_distanceModel;}
		virtual void SetDistanceModel(DistanceModel mdl) override {m_distanceModel = mdl;}
		
		virtual std::string GetDeviceName() const override {return "dummy";}
		virtual void PauseDeviceDSP() override {}
		virtual void ResumeDeviceDSP() override {}

		virtual std::vector<std::string> GetDevices() override {return {"dummy"};}
		virtual std::string GetDefaultDeviceName() override {return "dummy";}

		// HRTF
		virtual std::vector<std::string> GetHRTFNames() const override {return {};}
		virtual std::string GetCurrentHRTF() const override {return "";}
		virtual bool IsHRTFEnabled() const override {return false;}
		virtual void SetHRTF(uint32_t id) override {}
		virtual void DisableHRTF() override {}

		virtual uint32_t GetMaxAuxiliaryEffectsPerSource() const {return 0;}
	private:
		virtual ISoundBuffer *DoLoadSound(const std::string &path,bool bConvertToMono=false,bool bAsync=true) override;
		virtual PSoundChannel CreateChannel(ISoundBuffer &buffer) override;
		virtual PSoundChannel CreateChannel(Decoder &decoder) override {return nullptr;}
		virtual std::unique_ptr<IListener> CreateListener() override;
	};

	class DummySoundChannel
		: public ISoundChannel
	{
	public:
		enum class State : uint8_t
		{
			Initial = 0,
			Playing,
			Stopped,
			Paused
		};
		DummySoundChannel(ISoundSystem &system,ISoundBuffer &buffer)
			: ISoundChannel{system,buffer}
		{}
		DummySoundChannel(ISoundSystem &system,Decoder &decoder)
			: ISoundChannel{system,decoder}
		{}

		virtual void Play() override {m_state = State::Playing;}
		virtual void Stop() override {m_state = State::Stopped;}
		virtual void Pause() override {m_state = State::Paused;}
		virtual void Resume() override {Play();}
		virtual bool IsPlaying() const override {return m_state == State::Playing;}
		virtual bool IsPaused() const override {return m_state == State::Paused;}
		virtual void SetPriority(uint32_t priority) override {m_priority = priority;}
		virtual uint32_t GetPriority() const override {return m_priority;}
		virtual void SetFrameOffset(uint64_t offset) override {}
		virtual uint64_t GetFrameOffset(uint64_t *latency=nullptr) const override {return 0;}
		virtual void SetLooping(bool bLoop) override {m_looping = bLoop;}
		virtual bool IsLooping() const override {return m_looping;}

		virtual void SetPitch(float pitch) override {m_pitch = pitch;}
		virtual float GetPitch() const override {return m_pitch;}

		virtual void SetGain(float gain) override {m_gain = gain;}
		virtual float GetGain() const override {return m_gain;}

		virtual void SetGainRange(float minGain,float maxGain) override {m_minGain = minGain; m_maxGain = maxGain;}
		virtual std::pair<float,float> GetGainRange() const override {return {m_minGain,m_maxGain};}
		virtual float GetMinGain() const override {return m_minGain;}
		virtual float GetMaxGain() const override {return m_maxGain;}
		virtual void SetDistanceRange(float refDist,float maxDist) override {m_refDist = refDist; m_maxDist = maxDist;}
		virtual std::pair<float,float> GetDistanceRange() const override {return {m_refDist,m_maxDist};}
		virtual void SetPosition(const Vector3 &pos) override {m_pos = pos;}
		virtual Vector3 GetPosition() const override {return m_pos;}
		virtual void SetVelocity(const Vector3 &vel) override {m_velocity = vel;}
		virtual Vector3 GetVelocity() const override {return m_velocity;}

		virtual void SetDirection(const Vector3 &dir) override {m_direction = dir;}
		virtual Vector3 GetDirection() const override {return m_direction;}

		virtual void SetOrientation(const Vector3 &at,const Vector3 &up) override {m_at = at; m_up = up;}
		virtual std::pair<Vector3,Vector3> GetOrientation() const override {return {m_at,m_up};}

		virtual void SetConeAngles(float inner,float outer) override {m_innerConeAngles = inner; m_outerConeAngles = outer;}
		virtual std::pair<float,float> GetConeAngles() const override {return {m_innerConeAngles,m_outerConeAngles};}
		virtual void SetOuterConeGains(float gain,float gainHF=1.f) override {m_outerConeGain = gain; m_outerConeGainHf = gainHF;}
		virtual std::pair<float,float> GetOuterConeGains() const override {return {m_outerConeGain,m_outerConeGainHf};}
		virtual float GetOuterConeGain() const override {return m_outerConeGain;}
		virtual float GetOuterConeGainHF() const override {return m_outerConeGainHf;}

		virtual void SetRolloffFactors(float factor,float roomFactor=0.f) override {m_rolloffFactor = factor; m_roomRolloffFactor = roomFactor;}
		virtual std::pair<float,float> GetRolloffFactors() const override {return {m_rolloffFactor,m_roomRolloffFactor};}
		virtual float GetRolloffFactor() const override {return m_rolloffFactor;}
		virtual float GetRoomRolloffFactor() const override {return m_roomRolloffFactor;}

		virtual void SetDopplerFactor(float factor) override {m_dopplerFactor = factor;}
		virtual float GetDopplerFactor() const override {return m_dopplerFactor;}

		virtual void SetRelative(bool bRelative) override {m_relative = bRelative;}
		virtual bool IsRelative() const override {return m_relative;}

		virtual void SetRadius(float radius) override {m_radius = radius;}
		virtual float GetRadius() const override {return m_radius;}

		virtual void SetStereoAngles(float leftAngle,float rightAngle) override {m_stereoLeftAngle = leftAngle; m_stereoRightAngle = rightAngle;}
		virtual std::pair<float,float> GetStereoAngles() const override {return {m_stereoLeftAngle,m_stereoRightAngle};}
		virtual void SetAirAbsorptionFactor(float factor) override {m_airAbsorptionFactor = factor;}
		virtual float GetAirAbsorptionFactor() const override {return m_airAbsorptionFactor;}

		virtual void SetGainAuto(bool directHF,bool send,bool sendHF) override {m_directGainHfAuto = directHF; m_sendGainAuto = send; m_sendGainHfAuto = sendHF;}
		virtual std::tuple<bool,bool,bool> GetGainAuto() const override {return {m_directGainHfAuto,m_sendGainAuto,m_sendGainHfAuto};}
		virtual bool GetDirectGainHFAuto() const override {return m_directGainHfAuto;}
		virtual bool GetSendGainAuto() const override {return m_sendGainAuto;}
		virtual bool GetSendGainHFAuto() const override {return m_sendGainHfAuto;}

		virtual void SetDirectFilter(const EffectParams &params) override {}
		virtual void SetEffectParameters(uint32_t slotId,const EffectParams &params) override {}
	private:
		virtual void DoAddEffect(IAuxiliaryEffectSlot &slot,uint32_t slotId,const EffectParams &params) override {}
		virtual void DoRemoveInternalEffect(uint32_t slotId) override {}
		virtual void DoRemoveEffect(uint32_t slotId) override {}
		bool m_looping = false;
		bool m_relative = false;
		float m_refDist = 0.f;
		float m_maxDist = util::pragma::metres_to_units(1.0);
		uint32_t m_priority = 0;
		float m_radius = 0.f;
		float m_pitch = 1.f;
		float m_gain = 1.f;
		float m_outerConeGain = 1.f;
		float m_outerConeGainHf = 1.f;
		float m_dopplerFactor = 1.f;
		float m_rolloffFactor = 1.f;
		float m_roomRolloffFactor = 1.f;
		float m_airAbsorptionFactor = 1.f;
		float m_stereoLeftAngle = 0.f;
		float m_stereoRightAngle = 0.f;
		bool m_directGainHfAuto = false;
		bool m_sendGainAuto = false;
		bool m_sendGainHfAuto = false;
		float m_minGain = 0.f;
		float m_maxGain = 1.f;
		Vector3 m_pos {};
		Vector3 m_velocity {};
		Vector3 m_direction {0.f,0.f,0.f};
		Vector3 m_at {};
		Vector3 m_up {};
		float m_innerConeAngles = 0.f;
		float m_outerConeAngles = 0.f;
		State m_state = State::Initial;
	};

	class DummyListener
		: public IListener
	{
	public:
		DummyListener(al::ISoundSystem &system)
			: IListener{system}
		{}
	private:
		virtual void DoSetMetersPerUnit(float mu) override {}
	};

	class DummySoundBuffer
		: public ISoundBuffer
	{
	public:
		virtual uint32_t GetFrequency() const override {return 0;}
		virtual ChannelConfig GetChannelConfig() const override {return ChannelConfig::Mono;}
		virtual SampleType GetSampleType() const override {return SampleType::Float32;}
		virtual uint64_t GetLength() const override {return 0;}
		virtual std::pair<uint64_t,uint64_t> GetLoopFramePoints() const override {return {};}

		virtual bool IsReady() const override {return true;}

		virtual uint32_t GetSize() const override {return 0;}
		virtual void SetLoopFramePoints(uint32_t start,uint32_t end) override {}
		virtual void SetLoopTimePoints(float tStart,float tEnd) override {}

		virtual std::string GetName() const override {return "";}
		virtual bool IsInUse() const override {return false;}
	};
};

al::DummySoundSystem::DummySoundSystem(float metersPerUnit)
	: ISoundSystem{metersPerUnit}
{}

al::PSoundChannel al::DummySoundSystem::CreateChannel(ISoundBuffer &buffer)
{
	return std::make_shared<DummySoundChannel>(*this,buffer);
}
al::ISoundBuffer *al::DummySoundSystem::DoLoadSound(const std::string &path,bool bConvertToMono,bool bAsync)
{
	return new DummySoundBuffer{};
}
std::unique_ptr<al::IListener> al::DummySoundSystem::CreateListener()
{
	return std::make_unique<DummyListener>(*this);
}

extern "C"
{
    DLLEXPORT bool initialize_audio_api(float metersPerUnit,std::shared_ptr<al::ISoundSystem> &outSoundSystem,std::string &errMsg)
	{
		outSoundSystem = std::shared_ptr<al::DummySoundSystem>(new al::DummySoundSystem{metersPerUnit},[](al::DummySoundSystem *sys) {
			sys->OnRelease();
			delete sys;
		});
		return outSoundSystem != nullptr;
	}
};
