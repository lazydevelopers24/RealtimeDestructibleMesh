#include "BulletClusterComponent.h"
#include "Components/RealtimeDestructibleMeshComponent.h"
#include "TimerManager.h"

UBulletClusterComponent::UBulletClusterComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UBulletClusterComponent::BeginPlay()
{
	Super::BeginPlay();
}

void UBulletClusterComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{ 
	ClearPendingRequests();
	
	Super::EndPlay(EndPlayReason);
}

void UBulletClusterComponent::Init(float InMergeDistance, float InMaxCluserRadius, int InClusterCountThreshold, float InRaidusOffset)
{
	MergeDistanceThreshold = InMergeDistance;
	MaxClusterRadius = InMaxCluserRadius;
	ClusterCountThreshold = InClusterCountThreshold;
	ClusterRadiusOffset = InRaidusOffset;
}

void UBulletClusterComponent::SetOwnerMesh(URealtimeDestructibleMeshComponent* InOwnerMesh)
{
	OwnerMesh = InOwnerMesh;
}

void UBulletClusterComponent::RegisterRequest(const FVector& ImpactPoint, const FVector& ImpactNormal, const float Radius, int32 ChunkIndex)
{
	// 요청 추가
	FPendingClusteringRequest NewRequest;
	NewRequest.ImpactPoint  = ImpactPoint;
	NewRequest.ImpactNormal = ImpactNormal;
	NewRequest.Radius = Radius;
	NewRequest.ChunkIndex = ChunkIndex;
	PendingRequests.Add(NewRequest);
	
	// 타이머 시작
	if (!bTimerActive && GetWorld())
	{
		GetWorld()->GetTimerManager().SetTimer(
			ClusterTimerHandle,
			this,
			&UBulletClusterComponent::OnClusterWindowExpired,
			ClusterWindowTime,
			false);

		bTimerActive = true;
	}
	
}


void UBulletClusterComponent::OnClusterWindowExpired()
{
	bTimerActive = false;
	
	// 임계 Request 이상 쌓이지 않으면 초기화 
	if (PendingRequests.Num() < ClusterCountThreshold)
	{
		// 버퍼 클리어
		ClearPendingRequests();
		return;
	}
	
	//군집화 수행
	TArray<FBulletCluster> Clusters = ProcessClustering();
	 
	// 파괴
	if (Clusters.Num() > 0)
	{
	ExecuteDestruction(Clusters);
	}

	// 버퍼 클리어
	ClearPendingRequests();
	
}

TArray<FBulletCluster> UBulletClusterComponent::ProcessClustering()
{
	TArray<FBulletCluster> ResultClusters;
	int32 N = PendingRequests.Num();


	// 임계치 이상 안넘으면 return
	if (N < ClusterCountThreshold)
	{
		return ResultClusters;
	}

	// Union - Find
	FUnionFind ClusterUF;
	ClusterUF.Init(N);

	// 거리기반 Union
	for (int32 i = 0; i < N; ++i)
	{
		for (int32 j = i + 1; j < N; ++j)
		{
			float Dist = FVector::Dist(
			PendingRequests[i].ImpactPoint,
			PendingRequests[j].ImpactPoint
			);

			if (Dist <= MergeDistanceThreshold)
			{
				ClusterUF.Union(i, j);
			}
		}
	}

	// 클러스터 그룹핑
	TMap<int32, FBulletCluster> RootToCluster;

	for (int32 i = 0; i < N; ++i)
	{
		int32 Root = ClusterUF.Find(i);
		const FPendingClusteringRequest& Req = PendingRequests[i];

		FBulletCluster* FoundCluster = RootToCluster.Find(Root);

		// 아직 등록이 안되어있다면
		if (!FoundCluster)
		{
			FBulletCluster NewCluster;
			NewCluster.Init(Req.ImpactPoint, Req.ImpactNormal, Req.Radius, Req.ChunkIndex); 
			RootToCluster.Add(Root, NewCluster);
		}
		else
		{
			// Cluster에 새로운 총알이 추가 되었을 때,
			// 생기는 원의 반지름 크기를 예측해서 넣을지 말지 정한다.
			float PredicatedRauius = FoundCluster->PredictRadius(Req.ImpactPoint, Req.Radius);

			if (PredicatedRauius <= MaxClusterRadius)
			{
				FoundCluster->AddMember(Req.ImpactPoint, Req.ImpactNormal, Req.Radius, Req.ChunkIndex);
			}

		}
	}

	for (auto& Pair : RootToCluster)
	{
		if (Pair.Value.MemberPoints.Num() < ClusterCountThreshold)
		{
			continue;
		}

		ResultClusters.Add(Pair.Value);
	}

	return ResultClusters;
}

void UBulletClusterComponent::ExecuteDestruction(const TArray<FBulletCluster>& Clusters)
{

	URealtimeDestructibleMeshComponent* Mesh = OwnerMesh.Get();
	if (!Mesh || !IsValid(Mesh)) return;
	
	for (const FBulletCluster& Cluster : Clusters)
	{
		float FinalRadius = Cluster.Radius * ClusterRadiusOffset;


		TArray<int32> AffectedChunks;
		Mesh->FindChunksInRadius(Cluster.Center, FinalRadius, AffectedChunks);

		if (AffectedChunks.Num() == 0) continue;

		// 모든 청크가 동일한 Center를 사용하여 높이가 일관되게 유지됨
		for (int32 ChunkIndex : AffectedChunks)
	{
		FRealtimeDestructionRequest Request;
			Request.ImpactPoint = Cluster.Center;  // 모든 청크에 동일한 Center 사용
		Request.ImpactNormal = Cluster.Normal;
		Request.ToolShape = EDestructionToolShape::Cylinder;
			Request.ShapeParams.Radius = FinalRadius;
			Request.ChunkIndex = ChunkIndex;

		Request.ToolMeshPtr = Mesh->CreateToolMeshPtrFromShapeParams(
				Request.ToolShape, Request.ShapeParams);

		Mesh->ExecuteDestructionInternal(Request);
	}
	} 
}

void UBulletClusterComponent::ClearPendingRequests()
{
	// 타이머도 초기화
	if (bTimerActive && GetWorld())
	{
		GetWorld()->GetTimerManager().ClearTimer(ClusterTimerHandle);
		bTimerActive = false;
	}

	// 대기열 초기화
	PendingRequests.Empty();


}
