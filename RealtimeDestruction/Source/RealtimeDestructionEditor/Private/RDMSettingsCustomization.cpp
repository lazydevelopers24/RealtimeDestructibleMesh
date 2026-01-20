#include "RDMSettingsCustomization.h"
#include "RealtimeDestruction/Public/Settings/RDMSetting.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SSpinBox.h"
#include "HAL/PlatformMisc.h"

#define LOCTEXT_NAMESPACE "RDMSettingsCustomization"

TSharedRef<IDetailCustomization> FRdmSettingsCustomization::MakeInstance()
{
	return MakeShareable(new FRdmSettingsCustomization());
}

void FRdmSettingsCustomization::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
	// Settings 객체 가져오기
	TArray<TWeakObjectPtr<UObject>> Objects;
	DetailBuilder.GetObjectsBeingCustomized(Objects);

	if (Objects.Num() == 0)
	{
		return;
	}
	
	SettingsPtr = Cast<URDMSetting>(Objects[0].Get());
	if (!SettingsPtr.IsValid())
	{
		return;
	}

	// Thread Settings 카테고리 가져오기
	IDetailCategoryBuilder& Category = DetailBuilder.EditCategory("Thread Settings");

	int32 SystemThreads = FPlatformMisc::NumberOfCoresIncludingHyperthreads();

	Category.AddCustomRow(LOCTEXT("SystemThreads" , "System Threads"))
	.NameContent()
	[
		SNew(STextBlock)
		.Text(LOCTEXT("SystemThreadsLabel", "System Total Number Of Threads"))
		.Font(IDetailLayoutBuilder::GetDetailFont())
	]
	.ValueContent()
	[
		SNew(STextBlock)
		.Text(FText::Format(LOCTEXT("SystemThreadValue", "{0} threads"), FText::AsNumber(SystemThreads)))
		.Font(IDetailLayoutBuilder::GetDetailFont())
		
	];

	// 계산 결과 표시
	Category.AddCustomRow(LOCTEXT("Calculated Threads" , "Calculated Threads"))
	.Visibility(TAttribute<EVisibility>::Create([this]()
	{
		return (SettingsPtr.IsValid() && SettingsPtr->ThreadMode == ERDMThreadMode::Percentage) ? EVisibility::Visible : EVisibility::Collapsed;
	} ))
	.NameContent()
	[
		SNew(STextBlock)
		.Text(LOCTEXT("CalculatedLabel", "Number Of Threads To Use"))
		.Font(IDetailLayoutBuilder::GetDetailFont())
		.ColorAndOpacity(FSlateColor(FLinearColor::Green))
	]
	.ValueContent()
	[
		SAssignNew(ResultTextBlock, STextBlock)
		.Text_Lambda([this]()
		{
			if (SettingsPtr.IsValid())
			{
				int32 Result = SettingsPtr->GetEffectiveThreadCount();
				return FText::Format(LOCTEXT("Calculated Value", "{0} threads"), FText::AsNumber(Result));
			}
			return FText::GetEmpty();
		} )
	  .Font(IDetailLayoutBuilder::GetDetailFontBold())
	  .ColorAndOpacity(FSlateColor(FLinearColor::Green)) 
	];
 }

#undef LOCTEXT_NAMESPACE