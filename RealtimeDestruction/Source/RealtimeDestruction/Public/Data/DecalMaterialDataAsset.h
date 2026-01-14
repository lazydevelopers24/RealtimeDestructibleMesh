#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "DecalMaterialDataAsset.generated.h"
 
USTRUCT(BlueprintType)
struct REALTIMEDESTRUCTION_API FDecalSizeConfig 
{
	GENERATED_BODY()
 
	/** Decal Material */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Decal")
	TObjectPtr<UMaterialInterface> DecalMaterial = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector DecalSize = FVector(1.0f, 10.0f, 10.f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FVector LocationOffset = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FRotator RotationOffset = FRotator::ZeroRotator;

	// 유효성 검사 함수 (기존에 있다면 유지)
	bool IsValid() const { return DecalMaterial != nullptr; } 
};

USTRUCT(BlueprintType)
struct REALTIMEDESTRUCTION_API FProjectileDecalConfig
{
	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Decal")
	FName ConfigID = NAME_None;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Decal")
	TMap<FName,	FDecalSizeConfig> SurfaceConfigs;
};

UCLASS(BlueprintType)
class REALTIMEDESTRUCTION_API UDecalMaterialDataAsset : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	/** 총알 종류별 Decal 설정 목록 */ 
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Decal")
	TArray<FProjectileDecalConfig> ProjectileConfigs;
public:
	UFUNCTION(BlueprintCallable, Category = "Decal")
	bool GetConfig(FName ConfigID, FName SurfaceType, FDecalSizeConfig& OutConfig ) const;
 
	const FProjectileDecalConfig* FindProjectileConfig(FName ConfigID) const; 
	TArray<FName> GetAllConfigIDs() const;
	
	/** 보뮤하고 있는 Key의 수 */
	UFUNCTION(BlueprintCallable, Category = "Decal")
	int32 GetConfigCount() { return ProjectileConfigs.Num(); };
	
#if WITH_EDITORONLY_DATA
	UPROPERTY()
	FName CurrentEditingKey = NAME_None;
	
	// Tool Shape
	UPROPERTY()
	FVector ToolShapeLocationInEditor = FVector::ZeroVector;
	
	UPROPERTY()
	FRotator ToolShapeRotationInEditor = FRotator::ZeroRotator;

	UPROPERTY()
	float SphereRadiusInEditor = 10.0f;

	UPROPERTY()
	float CylinderRadiusInEditor = 10.0f;
	
	UPROPERTY()
	float CylinderHeightInEditor = 10.0f;

	UPROPERTY()
	TSoftObjectPtr<UStaticMesh> PreviewMeshInEditor = nullptr;

	UPROPERTY()
	FVector PreviewMeshLocationInEditor = FVector::ZeroVector;

	UPROPERTY()
	FRotator PreviewMeshRotationInEditor = FRotator::ZeroRotator;
	
	UPROPERTY()
	FVector PreviewMeshScaleInEditor = FVector::OneVector;
	
#endif
};
