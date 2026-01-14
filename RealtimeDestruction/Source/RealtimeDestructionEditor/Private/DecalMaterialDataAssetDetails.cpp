#include "DecalMaterialDataAssetDetails.h"

#include "DecalSizeEditorWindow.h"
#include "DetailCategoryBuilder.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "Data/DecalMaterialDataAsset.h"

TSharedRef<IDetailCustomization> FDecalMaterialDataAssetDetails::MakeInstance()
{
	return MakeShareable(new FDecalMaterialDataAssetDetails);
}

void FDecalMaterialDataAssetDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
	TArray<TWeakObjectPtr<UObject>> Objects;
	DetailBuilder.GetObjectsBeingCustomized(Objects);

	if ( Objects.Num() ==  0)
	{
		return;
	}

	TargetDataAsset = Cast<UDecalMaterialDataAsset>(Objects[0].Get());
	IDetailCategoryBuilder& DecalCategory = DetailBuilder.EditCategory("Decal");

	DecalCategory.AddCustomRow(FText::FromString("Open Decal Size Editor"))
		 .NameContent()
		 [
			 SNew(STextBlock)
			 .Text(FText::FromString("Decal Editor"))
			 .Font(IDetailLayoutBuilder::GetDetailFont())
		 ]
		 .ValueContent()
		 .MaxDesiredWidth(200.f)
		 [
			 SNew(SButton)
			 .Text(FText::FromString("Open Decal Size Editor"))
			 .HAlign(HAlign_Center)
			 .OnClicked(this, &FDecalMaterialDataAssetDetails::OnOpenEditorClicked)
		 ];
}

FReply FDecalMaterialDataAssetDetails::OnOpenEditorClicked()
{
	if (TargetDataAsset.IsValid())
	{
		SDecalSizeEditorWindow::OpenWindowForDataAsset(TargetDataAsset.Get()); 
	}

	return FReply::Handled();
}
