#pragma once

#include "CoreMinimal.h"
#include "IDetailCustomization.h"

class UDecalMaterialDataAsset;

class FDecalMaterialDataAssetDetails : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
private:
	TWeakObjectPtr<UDecalMaterialDataAsset> TargetDataAsset;
	FReply OnOpenEditorClicked();
	
};