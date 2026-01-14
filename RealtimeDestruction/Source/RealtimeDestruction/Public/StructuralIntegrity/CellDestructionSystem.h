// Copyright 2025. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "StructuralIntegrity/GridCellTypes.h"

/**
 * 셀 파괴 판정 시스템
 * BFS 기반 구조적 무결성 검사 및 파괴 판정
 */
class REALTIMEDESTRUCTION_API FCellDestructionSystem
{
public:
	//=========================================================================
	// 셀 파괴 판정
	//=========================================================================

	/**
	 * 파괴 형상으로 파괴된 셀 ID 계산
	 * 중심점 + 꼭지점 하이브리드 방식 사용
	 *
	 * @param Cache - 격자 캐시
	 * @param Shape - 파괴 형상 (양자화됨)
	 * @param MeshTransform - 메시 월드 트랜스폼
	 * @param DestroyedCells - 이미 파괴된 셀 집합 (제외용)
	 * @return 새로 파괴된 셀 ID 목록
	 */
	static TArray<int32> CalculateDestroyedCells(
		const FGridCellCache& Cache,
		const FQuantizedDestructionInput& Shape,
		const FTransform& MeshTransform,
		const TSet<int32>& DestroyedCells);

	/**
	 * 단일 셀이 파괴되었는지 판정
	 * Phase 1: 중심점 검사 (빠른 판정)
	 * Phase 2: 꼭지점 과반수 검사 (경계 케이스)
	 */
	static bool IsCellDestroyed(
		const FGridCellCache& Cache,
		int32 CellId,
		const FQuantizedDestructionInput& Shape,
		const FTransform& MeshTransform);

	/*
	 * <<<SubCell Level API>>>
	 * Destruction shape을 이용해 SubCell 레벨 파괴 수행
	 */
	static FDestructionResult ProcessCellDestructionWithSubCells(
		const FGridCellCache& Cache,
		const FQuantizedDestructionInput& Shape,
		const FTransform& MeshTransform,
		FCellState& InOutCellState);

	//=========================================================================
	// Cell 상태 조회
	//=========================================================================

	/**
	 * <<<SubCell Level API>>>
	 * Cell의 손상 수준 반환
	 * SubCell 상태에 따라 Intact/Damaged/Destroyed 반환
	 *
	 * @param CellId - 셀 ID
	 * @param CellState - 셀 상태
	 * @return 손상 수준
	 */
	static ECellDamageLevel GetCellDamageLevel(int32 CellId, const FCellState& CellState);
	
	//=========================================================================
	// 구조적 무결성 검사 (BFS)
	//=========================================================================

	/**
	 * 앵커에서 분리된 셀 찾기
	 *
	 * @param Cache - 격자 캐시
	 * @param DestroyedCells - 파괴된 셀 집합
	 * @return 분리된 셀 ID 집합
	 */
	static TSet<int32> FindDisconnectedCells(
		const FGridCellCache& Cache,
		const TSet<int32>& DestroyedCells);

	/**
	 * <<<SubCell Level API>>>
	 * 앵커에서 분리된 셀 찾기 (SubCell 레벨 연결성 검사)
	 *
	 * 파괴된 SubCell 주변만 검사하여 효율적으로 분리된 Cell을 찾음.
	 * SubCell 레벨 BFS로 Anchor 도달 여부를 판정.
	 *
	 * @param Cache - 격자 캐시
	 * @param CellState - 셀 상태 (SubCell 상태 포함)
	 * @param AffectedCells - 영향받은 Cell 목록 (검사 범위 제한)
	 * @return 분리된 셀 ID 집합
	 */
	static TSet<int32> FindDisconnectedCellsWithSubCells(
		const FGridCellCache& Cache,
		const FCellState& CellState,
		const TArray<int32>& AffectedCells);
	
	/**
	 * 분리된 셀들을 연결된 그룹으로 묶기
	 *
	 * @param Cache - 격자 캐시
	 * @param DisconnectedCells - 분리된 셀들
	 * @param DestroyedCells - 파괴된 셀들 (경계 제외용)
	 * @return 그룹별 셀 ID 목록
	 */
	static TArray<TArray<int32>> GroupDetachedCells(
		const FGridCellCache& Cache,
		const TSet<int32>& DisconnectedCells,
		const TSet<int32>& DestroyedCells);

	//=========================================================================
	// 유틸리티
	//=========================================================================

	/**
	 * 셀 그룹의 중심점 계산
	 */
	static FVector CalculateGroupCenter(
		const FGridCellCache& Cache,
		const TArray<int32>& CellIds,
		const FTransform& MeshTransform);

	/**
	 * 파편 초기 속도 계산 (폭발 방향)
	 */
	static FVector CalculateDebrisVelocity(
		const FVector& DebrisCenter,
		const TArray<FQuantizedDestructionInput>& DestructionInputs,
		float BaseSpeed = 500.0f);

	/**
	 * 경계 셀 판정 (파괴된 셀과 인접한 셀)
	 */
	static bool IsBoundaryCell(
		const FGridCellCache& Cache,
		int32 CellId,
		const TSet<int32>& DestroyedCells);
};

/**
 * 서버 파괴 배칭 처리 시스템
 * 16.6ms 주기로 파괴 이벤트를 모아서 처리
 */
class REALTIMEDESTRUCTION_API FDestructionBatchProcessor
{
public:
	/** 배칭 주기 (16.6ms = 60fps) */
	static constexpr float BatchInterval = 1.0f / 60.0f;

	FDestructionBatchProcessor();

	/**
	 * 파괴 요청 추가 (즉시 처리 안함, 큐에 저장)
	 */
	void QueueDestruction(const FCellDestructionShape& Shape);

	/**
	 * Tick 처리 (배칭 주기 확인)
	 * @return 배치 처리가 수행되었으면 true
	 */
	bool Tick(float DeltaTime);

	/**
	 * 강제로 현재 큐 처리 (즉시 처리 필요시)
	 */
	void FlushQueue();

	/**
	 * 배치 처리 결과 가져오기 (처리 후 호출)
	 */
	const FBatchedDestructionEvent& GetLastBatchResult() const { return LastBatchResult; }

	/**
	 * 처리할 파괴가 있는지 확인
	 */
	bool HasPendingDestructions() const { return PendingDestructions.Num() > 0; }

	/**
	 * 처리 컨텍스트 설정 (배치 처리 전 호출 필요)
	 */
	void SetContext(
		const FGridCellCache* InCache,
		FCellState* InCellState,
		const FTransform& InMeshTransform);

private:
	/** 실제 배치 처리 */
	void ProcessBatch();

	/** 16.6ms 동안 쌓인 파괴 요청들 */
	TArray<FQuantizedDestructionInput> PendingDestructions;

	/** 타이머 누적 */
	float AccumulatedTime;

	/** 마지막 배치 결과 */
	FBatchedDestructionEvent LastBatchResult;

	/** 처리 컨텍스트 */
	const FGridCellCache* CachePtr;
	FCellState* CellStatePtr;
	FTransform MeshTransform;

	/** 파편 ID 카운터 */
	int32 DebrisIdCounter;
};
