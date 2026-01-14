// Copyright 2025. All Rights Reserved.

#include "StructuralIntegrity/CellDestructionSystem.h"
#include "StructuralIntegrity/SubCellProcessor.h"

//=============================================================================
// FCellDestructionSystem - SubCell Level API
//=============================================================================

FDestructionResult FCellDestructionSystem::ProcessCellDestructionWithSubCells(
	const FGridCellCache& Cache,
	const FQuantizedDestructionInput& Shape,
	const FTransform& MeshTransform,
	FCellState& InOutCellState)
{
	FDestructionResult Result;

	if (!Cache.IsValid())
	{
		return Result;
	}

	// 1. SubCellProcessor를 통한 SubCell 파괴 처리
	TArray<int32> AffectedCells;
	TMap<int32, TArray<int32>> NewlyDeadSubCells;

	FSubCellProcessor::ProcessSubCellDestruction(
		Shape,
		MeshTransform,
		Cache,
		InOutCellState,
		AffectedCells,
		&NewlyDeadSubCells
	);

	// 2. 결과 정리
	Result.AffectedCells = MoveTemp(AffectedCells);

	// NewlyDeadSubCells (TMap<int32, TArray<int32>>) → FDestructionResult.NewlyDeadSubCells (TMap<int32, FIntArray>)
	for (auto& Pair : NewlyDeadSubCells)
	{
		FIntArray SubCellArray;
		SubCellArray.Values = MoveTemp(Pair.Value);
		Result.DeadSubCellCount += SubCellArray.Num();
		Result.NewlyDeadSubCells.Add(Pair.Key, MoveTemp(SubCellArray));
	}

	// 3. 완전히 파괴된 Cell 수집 (SubCellProcessor 내부에서 이미 DestroyedCells에 추가됨)
	for (int32 CellId : Result.AffectedCells)
	{
		if (InOutCellState.DestroyedCells.Contains(CellId))
		{
			Result.NewlyDestroyedCells.Add(CellId);
		}
	}

	return Result;
}

ECellDamageLevel FCellDestructionSystem::GetCellDamageLevel(int32 CellId, const FCellState& CellState)
{
	// Destroyed 확인
	if (CellState.DestroyedCells.Contains(CellId))
	{
		return ECellDamageLevel::Destroyed;
	}

	// SubCell 상태 확인
	const FSubCell* SubCellState = CellState.SubCellStates.Find(CellId);
	if (!SubCellState)
	{
		return ECellDamageLevel::Intact;  // SubCell 상태 없음 = 손상 없음
	}

	if (SubCellState->IsFullyDestroyed())
	{
		return ECellDamageLevel::Destroyed;
	}

	return ECellDamageLevel::Damaged;
}

//=============================================================================
// FCellDestructionSystem - 셀 파괴 판정 (기존 Cell Level API)
//=============================================================================

TArray<int32> FCellDestructionSystem::CalculateDestroyedCells(
	const FGridCellCache& Cache,
	const FQuantizedDestructionInput& Shape,
	const FTransform& MeshTransform,
	const TSet<int32>& DestroyedCells)
{
	TArray<int32> NewlyDestroyed;

	for (int32 CellId = 0; CellId < Cache.GetTotalCellCount(); CellId++)
	{
		// 이미 파괴되었거나 존재하지 않는 셀은 스킵
		if (!Cache.GetCellExists(CellId) || DestroyedCells.Contains(CellId))
		{
			continue;
		}

		if (IsCellDestroyed(Cache, CellId, Shape, MeshTransform))
		{
			NewlyDestroyed.Add(CellId);
		}
	}

	return NewlyDestroyed;
}

bool FCellDestructionSystem::IsCellDestroyed(
	const FGridCellCache& Cache,
	int32 CellId,
	const FQuantizedDestructionInput& Shape,
	const FTransform& MeshTransform)
{
	// Phase 1: 중심점 검사 (빠른 판정)
	const FVector WorldCenter = Cache.IdToWorldCenter(CellId, MeshTransform);
	if (Shape.ContainsPoint(WorldCenter))
	{
		return true;
	}

	// Phase 2: 꼭지점 과반수 검사 (경계 케이스)
	const TArray<FVector> LocalVertices = Cache.GetCellVertices(CellId);
	int32 DestroyedVertices = 0;

	for (const FVector& LocalVertex : LocalVertices)
	{
		const FVector WorldVertex = MeshTransform.TransformPosition(LocalVertex);
		if (Shape.ContainsPoint(WorldVertex))
		{
			// 과반수 (4개) 도달 시 즉시 반환
			if (++DestroyedVertices >= 4)
			{
				return true;
			}
		}
	}

	return false;
}

//=============================================================================
// FCellDestructionSystem - 구조적 무결성 검사
//=============================================================================

TSet<int32> FCellDestructionSystem::FindDisconnectedCells(
	const FGridCellCache& Cache,
	const TSet<int32>& DestroyedCells)
{
	TSet<int32> Connected;
	TQueue<int32> Queue;

	// 1. 앵커에서 BFS 시작
	for (int32 CellId = 0; CellId < Cache.GetTotalCellCount(); CellId++)
	{
		if (Cache.GetCellExists(CellId) &&
		    Cache.GetCellIsAnchor(CellId) &&
		    !DestroyedCells.Contains(CellId))
		{
			Queue.Enqueue(CellId);
			Connected.Add(CellId);
		}
	}

	// 2. BFS 탐색
	while (!Queue.IsEmpty())
	{
		int32 Current;
		Queue.Dequeue(Current);

		for (int32 Neighbor : Cache.GetCellNeighbors(Current))
		{
			if (!DestroyedCells.Contains(Neighbor) &&
			    !Connected.Contains(Neighbor))
			{
				Connected.Add(Neighbor);
				Queue.Enqueue(Neighbor);
			}
		}
	}

	// 3. 연결되지 않은 셀 = 분리됨
	TSet<int32> Disconnected;
	for (int32 CellId = 0; CellId < Cache.GetTotalCellCount(); CellId++)
	{
		if (Cache.GetCellExists(CellId) &&
		    !DestroyedCells.Contains(CellId) &&
		    !Connected.Contains(CellId))
		{
			Disconnected.Add(CellId);
		}
	}

	return Disconnected;
}

TArray<TArray<int32>> FCellDestructionSystem::GroupDetachedCells(
	const FGridCellCache& Cache,
	const TSet<int32>& DisconnectedCells,
	const TSet<int32>& DestroyedCells)
{
	TArray<TArray<int32>> Groups;
	TSet<int32> Visited;

	for (int32 StartCell : DisconnectedCells)
	{
		if (Visited.Contains(StartCell))
		{
			continue;
		}

		// BFS로 연결된 분리 셀 그룹 찾기
		TArray<int32> Group;
		TQueue<int32> Queue;

		Queue.Enqueue(StartCell);
		Visited.Add(StartCell);

		while (!Queue.IsEmpty())
		{
			int32 Current;
			Queue.Dequeue(Current);
			Group.Add(Current);

			for (int32 Neighbor : Cache.GetCellNeighbors(Current))
			{
				if (DisconnectedCells.Contains(Neighbor) &&
				    !Visited.Contains(Neighbor))
				{
					Visited.Add(Neighbor);
					Queue.Enqueue(Neighbor);
				}
			}
		}

		Groups.Add(MoveTemp(Group));
	}

	return Groups;
}

//=============================================================================
// FCellDestructionSystem - 유틸리티
//=============================================================================

FVector FCellDestructionSystem::CalculateGroupCenter(
	const FGridCellCache& Cache,
	const TArray<int32>& CellIds,
	const FTransform& MeshTransform)
{
	if (CellIds.Num() == 0)
	{
		return FVector::ZeroVector;
	}

	FVector Sum = FVector::ZeroVector;
	for (int32 CellId : CellIds)
	{
		Sum += Cache.IdToWorldCenter(CellId, MeshTransform);
	}

	return Sum / CellIds.Num();
}

FVector FCellDestructionSystem::CalculateDebrisVelocity(
	const FVector& DebrisCenter,
	const TArray<FQuantizedDestructionInput>& DestructionInputs,
	float BaseSpeed)
{
	if (DestructionInputs.Num() == 0)
	{
		return FVector::ZeroVector;
	}

	// 가장 가까운 파괴 입력 찾기
	float MinDistSq = MAX_FLT;
	FVector ClosestCenter = FVector::ZeroVector;

	for (const auto& Input : DestructionInputs)
	{
		const FVector Center = FVector(Input.CenterMM.X, Input.CenterMM.Y, Input.CenterMM.Z) * 0.1f;
		const float DistSq = FVector::DistSquared(DebrisCenter, Center);

		if (DistSq < MinDistSq)
		{
			MinDistSq = DistSq;
			ClosestCenter = Center;
		}
	}

	// 폭발 방향으로 속도
	const FVector Direction = (DebrisCenter - ClosestCenter).GetSafeNormal();
	return Direction * BaseSpeed;
}

bool FCellDestructionSystem::IsBoundaryCell(
	const FGridCellCache& Cache,
	int32 CellId,
	const TSet<int32>& DestroyedCells)
{
	for (int32 Neighbor : Cache.GetCellNeighbors(CellId))
	{
		if (DestroyedCells.Contains(Neighbor))
		{
			return true;  // 파괴된 셀과 인접 = 경계
		}
	}
	return false;
}

//=============================================================================
// FDestructionBatchProcessor
//=============================================================================

FDestructionBatchProcessor::FDestructionBatchProcessor()
	: AccumulatedTime(0.0f)
	, CachePtr(nullptr)
	, CellStatePtr(nullptr)
	, MeshTransform(FTransform::Identity)
	, DebrisIdCounter(0)
{
}

void FDestructionBatchProcessor::QueueDestruction(const FCellDestructionShape& Shape)
{
	// 양자화하여 저장
	PendingDestructions.Add(FQuantizedDestructionInput::FromDestructionShape(Shape));
}

bool FDestructionBatchProcessor::Tick(float DeltaTime)
{
	AccumulatedTime += DeltaTime;

	if (AccumulatedTime >= BatchInterval && PendingDestructions.Num() > 0)
	{
		AccumulatedTime = 0.0f;
		ProcessBatch();
		return true;
	}

	return false;
}

void FDestructionBatchProcessor::FlushQueue()
{
	if (PendingDestructions.Num() > 0)
	{
		ProcessBatch();
		AccumulatedTime = 0.0f;
	}
}

void FDestructionBatchProcessor::SetContext(
	const FGridCellCache* InCache,
	FCellState* InCellState,
	const FTransform& InMeshTransform)
{
	CachePtr = InCache;
	CellStatePtr = InCellState;
	MeshTransform = InMeshTransform;
}

void FDestructionBatchProcessor::ProcessBatch()
{
	if (!CachePtr || !CellStatePtr)
	{
		UE_LOG(LogTemp, Warning, TEXT("FDestructionBatchProcessor: Context not set"));
		PendingDestructions.Empty();
		return;
	}

	// 결과 초기화
	LastBatchResult = FBatchedDestructionEvent();
	LastBatchResult.DestructionInputs = PendingDestructions;

	//=====================================================
	// Phase 1: 모든 파괴 입력으로 셀 판정
	//=====================================================
	TSet<int32> NewlyDestroyed;

	for (const auto& Input : PendingDestructions)
	{
		TArray<int32> Cells = FCellDestructionSystem::CalculateDestroyedCells(
			*CachePtr, Input, MeshTransform, CellStatePtr->DestroyedCells);

		for (int32 CellId : Cells)
		{
			NewlyDestroyed.Add(CellId);
		}
	}

	if (NewlyDestroyed.Num() == 0)
	{
		PendingDestructions.Empty();
		return;
	}

	//=====================================================
	// Phase 2: 셀 상태 업데이트
	//=====================================================
	for (int32 CellId : NewlyDestroyed)
	{
		CellStatePtr->DestroyedCells.Add(CellId);
	}

	//=====================================================
	// Phase 3: BFS 1회만 실행 (배칭의 핵심)
	//=====================================================
	TSet<int32> Disconnected = FCellDestructionSystem::FindDisconnectedCells(
		*CachePtr, CellStatePtr->DestroyedCells);

	TArray<TArray<int32>> DetachedGroups = FCellDestructionSystem::GroupDetachedCells(
		*CachePtr, Disconnected, CellStatePtr->DestroyedCells);

	//=====================================================
	// Phase 4: 분리된 셀들도 파괴 처리
	//=====================================================
	for (const auto& Group : DetachedGroups)
	{
		for (int32 CellId : Group)
		{
			CellStatePtr->DestroyedCells.Add(CellId);
		}
	}

	//=====================================================
	// Phase 5: 이벤트 생성
	//=====================================================
	for (int32 CellId : NewlyDestroyed)
	{
		LastBatchResult.DestroyedCellIds.Add((int16)CellId);
	}

	// 파편 정보 생성
	for (const auto& Group : DetachedGroups)
	{
		FDetachedDebrisInfo DebrisInfo;
		DebrisInfo.DebrisId = ++DebrisIdCounter;

		for (int32 CellId : Group)
		{
			DebrisInfo.CellIds.Add((int16)CellId);
			LastBatchResult.DestroyedCellIds.Add((int16)CellId);
		}

		DebrisInfo.InitialLocation = FCellDestructionSystem::CalculateGroupCenter(
			*CachePtr, Group, MeshTransform);

		DebrisInfo.InitialVelocity = FCellDestructionSystem::CalculateDebrisVelocity(
			DebrisInfo.InitialLocation, PendingDestructions);

		LastBatchResult.DetachedDebris.Add(DebrisInfo);
	}

	// 큐 초기화
	PendingDestructions.Empty();

	UE_LOG(LogTemp, Log, TEXT("FDestructionBatchProcessor: Processed %d destroyed cells, %d debris groups"),
		LastBatchResult.DestroyedCellIds.Num(), LastBatchResult.DetachedDebris.Num());
}

//=============================================================================
// FCellDestructionSystem - SubCell 레벨 연결성 검사
//=============================================================================

namespace SubCellBFSHelper
{
	/**
	 * 두 Cell 사이의 방향 계산
	 * @return 방향 인덱스 (0-5: -X, +X, -Y, +Y, -Z, +Z), -1이면 인접하지 않음
	 */
	int32 GetNeighborDirection(const FGridCellCache& Cache, int32 FromCellId, int32 ToCellId)
	{
		const FIntVector FromCoord = Cache.IdToCoord(FromCellId);
		const FIntVector ToCoord = Cache.IdToCoord(ToCellId);
		const FIntVector Diff = ToCoord - FromCoord;

		// 인접 Cell은 한 축으로만 ±1 차이
		if (Diff == FIntVector(-1, 0, 0)) return 0;  // -X
		if (Diff == FIntVector(+1, 0, 0)) return 1;  // +X
		if (Diff == FIntVector(0, -1, 0)) return 2;  // -Y
		if (Diff == FIntVector(0, +1, 0)) return 3;  // +Y
		if (Diff == FIntVector(0, 0, -1)) return 4;  // -Z
		if (Diff == FIntVector(0, 0, +1)) return 5;  // +Z

		return -1;  // 인접하지 않음
	}

	/**
	 * 경계 SubCell → 이웃 Cell의 대응 SubCell ID
	 * 예: +X 방향 이동 시, 현재 Cell의 (4,Y,Z) → 이웃 Cell의 (0,Y,Z)
	 */
	int32 GetCorrespondingBoundarySubCell(int32 SubCellId, int32 Direction)
	{
		const FIntVector Coord = SubCellIdToCoord(SubCellId);

		switch (Direction)
		{
		case 0:  // -X → 이웃 Cell의 +X 경계 (X=4)
			return SubCellCoordToId(SUBCELL_DIVISION - 1, Coord.Y, Coord.Z);
		case 1:  // +X → 이웃 Cell의 -X 경계 (X=0)
			return SubCellCoordToId(0, Coord.Y, Coord.Z);
		case 2:  // -Y → 이웃 Cell의 +Y 경계 (Y=4)
			return SubCellCoordToId(Coord.X, SUBCELL_DIVISION - 1, Coord.Z);
		case 3:  // +Y → 이웃 Cell의 -Y 경계 (Y=0)
			return SubCellCoordToId(Coord.X, 0, Coord.Z);
		case 4:  // -Z → 이웃 Cell의 +Z 경계 (Z=4)
			return SubCellCoordToId(Coord.X, Coord.Y, SUBCELL_DIVISION - 1);
		case 5:  // +Z → 이웃 Cell의 -Z 경계 (Z=0)
			return SubCellCoordToId(Coord.X, Coord.Y, 0);
		default:
			return -1;
		}
	}

	/**
	 * SubCell이 특정 방향의 경계에 있는지 확인
	 * 예: Direction=1 (+X) → SubCell의 X 좌표가 4인지 확인
	 */
	bool IsSubCellOnBoundary(int32 SubCellId, int32 Direction)
	{
		const FIntVector Coord = SubCellIdToCoord(SubCellId);

		switch (Direction)
		{
		case 0: return Coord.X == 0;                      // -X 경계
		case 1: return Coord.X == SUBCELL_DIVISION - 1;   // +X 경계
		case 2: return Coord.Y == 0;                      // -Y 경계
		case 3: return Coord.Y == SUBCELL_DIVISION - 1;   // +Y 경계
		case 4: return Coord.Z == 0;                      // -Z 경계
		case 5: return Coord.Z == SUBCELL_DIVISION - 1;   // +Z 경계
		default: return false;
		}
	}

	/**
	 * Cell의 첫 번째 살아있는 SubCell 찾기
	 * @return SubCellId, 없으면 -1
	 */
	int32 FindFirstAliveSubCell(int32 CellId, const FCellState& CellState)
	{
		if (CellState.DestroyedCells.Contains(CellId))
		{
			return -1;
		}

		const FSubCell* SubCellState = CellState.SubCellStates.Find(CellId);

		for (int32 SubCellId = 0; SubCellId < SUBCELL_COUNT; ++SubCellId)
		{
			bool bAlive = SubCellState ? SubCellState->IsSubCellAlive(SubCellId) : true;
			if (bAlive)
			{
				return SubCellId;
			}
		}

		return -1;
	}

	/**
	 * SubCell 레벨 BFS로 Anchor 도달 여부 확인
	 *
	 * @param Cache - 격자 캐시
	 * @param CellState - 셀 상태
	 * @param StartCellId - 시작 Cell
	 * @param ConfirmedConnected - 이미 Connected 확인된 Cell들
	 * @param OutVisitedCells - 방문한 Cell 집합 (출력)
	 * @return Anchor 도달 여부
	 */
	bool PerformSubCellBFS(
		const FGridCellCache& Cache,
		const FCellState& CellState,
		int32 StartCellId,
		const TSet<int32>& ConfirmedConnected,
		TSet<int32>& OutVisitedCells)
	{
		OutVisitedCells.Reset();

		// 시작 Cell의 살아있는 SubCell 찾기
		const int32 StartSubCell = FindFirstAliveSubCell(StartCellId, CellState);
		if (StartSubCell < 0)
		{
			// 살아있는 SubCell 없음 = 이미 완전 파괴된 Cell
			return false;
		}

		// BFS 큐: (CellId, SubCellId)
		TQueue<TPair<int32, int32>> Queue;

		// 방문한 SubCell 추적: (CellId << 16) | SubCellId
		// 참고: CellId가 65535 이하라고 가정 (일반적인 파괴 메시에서 충분)
		TSet<int32> VisitedSubCells;

		auto MakeKey = [](int32 CellId, int32 SubCellId) -> int32
		{
			return (CellId << 16) | SubCellId;
		};

		Queue.Enqueue(TPair<int32, int32>(StartCellId, StartSubCell));
		VisitedSubCells.Add(MakeKey(StartCellId, StartSubCell));
		OutVisitedCells.Add(StartCellId);

		while (!Queue.IsEmpty())
		{
			TPair<int32, int32> Current;
			Queue.Dequeue(Current);

			const int32 CurrCellId = Current.Key;
			const int32 CurrSubCellId = Current.Value;

			// Anchor Cell에 도달했는지 확인
			if (Cache.GetCellIsAnchor(CurrCellId))
			{
				return true;
			}

			// 이미 Connected 확인된 Cell에 도달
			if (ConfirmedConnected.Contains(CurrCellId))
			{
				return true;
			}

			// 현재 SubCell 좌표
			const FIntVector SubCoord = SubCellIdToCoord(CurrSubCellId);

			// 6방향 탐색
			for (int32 Dir = 0; Dir < 6; ++Dir)
			{
				const FIntVector NewSubCoord = SubCoord + FIntVector(
					DIRECTION_OFFSETS[Dir][0],
					DIRECTION_OFFSETS[Dir][1],
					DIRECTION_OFFSETS[Dir][2]
				);

				// 같은 Cell 내 이동 가능한지 확인
				if (NewSubCoord.X >= 0 && NewSubCoord.X < SUBCELL_DIVISION &&
					NewSubCoord.Y >= 0 && NewSubCoord.Y < SUBCELL_DIVISION &&
					NewSubCoord.Z >= 0 && NewSubCoord.Z < SUBCELL_DIVISION)
				{
					// 같은 Cell 내 이동
					const int32 NextSubCellId = SubCellCoordToId(NewSubCoord.X, NewSubCoord.Y, NewSubCoord.Z);
					const int32 Key = MakeKey(CurrCellId, NextSubCellId);

					if (!VisitedSubCells.Contains(Key))
					{
						if (CellState.IsSubCellAlive(CurrCellId, NextSubCellId))
						{
							VisitedSubCells.Add(Key);
							Queue.Enqueue(TPair<int32, int32>(CurrCellId, NextSubCellId));
						}
					}
				}
				else
				{
					// 경계를 넘어 다른 Cell로 이동 시도
					// 현재 SubCell이 해당 방향 경계에 있는지 확인
					if (!IsSubCellOnBoundary(CurrSubCellId, Dir))
					{
						continue;
					}

					// 해당 방향의 이웃 Cell 찾기
					const FIntVector CurrCellCoord = Cache.IdToCoord(CurrCellId);
					const FIntVector NeighborCoord = CurrCellCoord + FIntVector(
						DIRECTION_OFFSETS[Dir][0],
						DIRECTION_OFFSETS[Dir][1],
						DIRECTION_OFFSETS[Dir][2]
					);

					if (!Cache.IsValidCoord(NeighborCoord))
					{
						continue;
					}

					const int32 NeighborCellId = Cache.CoordToId(NeighborCoord);

					// 이웃 Cell이 유효한지 확인
					if (!Cache.GetCellExists(NeighborCellId) ||
						CellState.DestroyedCells.Contains(NeighborCellId))
					{
						continue;
					}

					// 이웃 Cell의 대응 경계 SubCell
					const int32 NeighborSubCellId = GetCorrespondingBoundarySubCell(CurrSubCellId, Dir);
					const int32 Key = MakeKey(NeighborCellId, NeighborSubCellId);

					if (!VisitedSubCells.Contains(Key))
					{
						// 이웃 Cell의 경계 SubCell이 살아있는지 확인
						if (CellState.IsSubCellAlive(NeighborCellId, NeighborSubCellId))
						{
							VisitedSubCells.Add(Key);
							Queue.Enqueue(TPair<int32, int32>(NeighborCellId, NeighborSubCellId));
							OutVisitedCells.Add(NeighborCellId);
						}
					}
				}
			}
		}

		// Anchor에 도달하지 못함
		return false;
	}
}

TSet<int32> FCellDestructionSystem::FindDisconnectedCellsWithSubCells(
	const FGridCellCache& Cache,
	const FCellState& CellState,
	const TArray<int32>& AffectedCells)
{
	using namespace SubCellBFSHelper;

	TSet<int32> Disconnected;
	TSet<int32> ConfirmedConnected;
	TSet<int32> Processed;

	// 1. 분리 후보 수집: AffectedCells + 인접 Cell들
	TSet<int32> Candidates;
	for (int32 CellId : AffectedCells)
	{
		// 파괴된 Cell은 제외
		if (CellState.DestroyedCells.Contains(CellId))
		{
			continue;
		}

		Candidates.Add(CellId);

		// 인접 Cell들도 추가 (경계 연결이 끊겼을 수 있음)
		for (int32 NeighborId : Cache.GetCellNeighbors(CellId).Values)
		{
			if (!CellState.DestroyedCells.Contains(NeighborId))
			{
				Candidates.Add(NeighborId);
			}
		}
	}

	// 2. 각 후보 Cell에서 SubCell BFS
	for (int32 CandidateCell : Candidates)
	{
		if (Processed.Contains(CandidateCell))
		{
			continue;
		}

		// SubCell 레벨 BFS 수행
		TSet<int32> VisitedCells;
		const bool bReachedAnchor = PerformSubCellBFS(
			Cache, CellState, CandidateCell, ConfirmedConnected, VisitedCells);

		// 방문한 모든 Cell 처리
		for (int32 VisitedCell : VisitedCells)
		{
			Processed.Add(VisitedCell);

			if (bReachedAnchor)
			{
				ConfirmedConnected.Add(VisitedCell);
			}
			else
			{
				Disconnected.Add(VisitedCell);
			}
		}
	}

	return Disconnected;
}
