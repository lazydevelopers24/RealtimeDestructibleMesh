#include "Settings/RDMSetting.h"
#include "HAL/PlatformMisc.h"
#include "Misc/TypeContainer.h"

URDMSetting::URDMSetting()
{
	ThreadMode = ERDMThreadMode::Absolute;

	MaxThreadCount = 8;
	ThreadPercentage = 50;
}

URDMSetting* URDMSetting::Get()
{
	return GetMutableDefault<URDMSetting>();
}

int32 URDMSetting::	GetEffectiveThreadCount() const
{
	if (ThreadMode == ERDMThreadMode::Absolute)
	{
		return FMath::Clamp(MaxThreadCount, 1, GetSystemThreadCount());
	}
	else
	{
		int32 SystemThreads = GetSystemThreadCount();
		int32 CalculatedThreads = FMath::CeilToInt(SystemThreads *ThreadPercentage / 100.0f);
		return FMath::Clamp(CalculatedThreads, 1, SystemThreads);
	}
}

int32 URDMSetting::GetSystemThreadCount()
{
	return FPlatformMisc::NumberOfCoresIncludingHyperthreads();
}

