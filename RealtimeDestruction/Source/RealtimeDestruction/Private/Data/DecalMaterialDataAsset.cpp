#include "Data/DecalMaterialDataAsset.h" 

bool UDecalMaterialDataAsset::GetConfig(FName ConfigID, FName SurfaceType, FDecalSizeConfig& OutConfig) const
{

	// ConfigID로 못찾으면 return false
    const FProjectileDecalConfig* ProjectileConfig = FindProjectileConfig(ConfigID);
	if (!ProjectileConfig)
	{
		return false;
	}

	// SurfaceType으로 DecalConfig 찾기
	if ( const FDecalSizeConfig* Found = ProjectileConfig->SurfaceConfigs.Find(SurfaceType))
	{
		OutConfig = *Found;
		return true;
	}

	// DecalConfig 못 찾았으면 default 값을 할당 시도
	if (SurfaceType != "Default")
	{
		if (const FDecalSizeConfig* DefaultConfig = ProjectileConfig->SurfaceConfigs.Find("Default"))
		{
			OutConfig = *DefaultConfig;
			return true;
		}
	}

	return false;
}

const FProjectileDecalConfig* UDecalMaterialDataAsset::FindProjectileConfig(FName ConfigID) const
{
	for (const FProjectileDecalConfig& Config : ProjectileConfigs)
      {
          if (Config.ConfigID == ConfigID)
          {
              return &Config;
          }
      }
      return nullptr;
}

TArray<FName> UDecalMaterialDataAsset::GetAllConfigIDs() const
{  
	TArray<FName> Result;
	for (const FProjectileDecalConfig& Config : ProjectileConfigs)
	{
		Result.Add(Config.ConfigID);
	}
	return Result;
}
  