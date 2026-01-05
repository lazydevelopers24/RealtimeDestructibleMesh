// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "CoreMinimal.h"
#include "Components/DynamicMeshComponent.h"

namespace UE::Geometry
{
	class FDynamicMesh3;
	class FMeshConnectedComponents;
	struct FIndex3i;
}

using UE::Geometry::FDynamicMesh3;
using UE::Geometry::FMeshConnectedComponents;
using UE::Geometry::FIndex3i;

// 절단 평면의 유한 사각형 영역(슬라이스 면)을 표현
struct FChunkDivisionPlaneRect
{
	FVector PlaneOrigin = FVector::ZeroVector;     // 평면 위의 기준점 (Plane 방정식의 한 점)
	FVector PlaneNormal = FVector::UpVector;       // 평면의 법선 (정규화 권장)
	FVector RectCenter = FVector::ZeroVector;      // 사각형 중심점 (반드시 평면 위)
	FVector RectAxisU = FVector::ForwardVector;    // 평면 내 U 축 (정규화, PlaneNormal과 직교)
	FVector RectAxisV = FVector::RightVector;      // 평면 내 V 축 (정규화, PlaneNormal과 직교)
	FVector2D HalfExtents = FVector2D::ZeroVector; // 사각형 반 크기 (U/V 방향 길이의 절반)
	int32 ChunkA = INDEX_NONE;                     // 평면 한쪽에 있는 청크 ID
	int32 ChunkB = INDEX_NONE;                     // 평면 반대쪽에 있는 청크 ID
};

// 평면 사각형에 접한 삼각형의 2D 투영 데이터
struct FChunkBoundaryTriangle2D
{
	FVector2D P0 = FVector2D::ZeroVector; // 평면 UV 좌표 0
	FVector2D P1 = FVector2D::ZeroVector; // 평면 UV 좌표 1
	FVector2D P2 = FVector2D::ZeroVector; // 평면 UV 좌표 2
	FBox2D Bounds;                        // 삼각형 2D AABB
};

// 그래프 인접 정보: 현재 노드와 연결된 상대 노드
struct FChunkCellNeighbor
{
	int32 ChunkId = INDEX_NONE;            // 연결된 상대 Chunk ID
	int32 CellId = INDEX_NONE;             // 연결된 상대 Cell ID (상대 Chunk 내부에서 고유)
	int32 DivisionPlaneIndex = INDEX_NONE; // 연결 기준이 되는 분할 평면 인덱스 (없으면 INDEX_NONE)
};

// 그래프 노드: Chunk/Cell 단위 정보
struct FChunkCellNode
{
	int32 ChunkId = INDEX_NONE;               // Chunk ID (하나의 UDynamicMeshComponent에 대응)
	int32 CellId = INDEX_NONE;                // Cell ID (해당 Chunk 내부에서만 고유, FMeshConnectedComponent에 대응)
	TArray<FChunkCellNeighbor> Neighbors;     // 인접 노드 목록
	bool bIsAnchor = false;                   // Anchor 여부
};

// 한 Chunk 내부의 Cell(ConnectedComponent) 캐시
struct FChunkCellCache
{
	int32 ChunkId = INDEX_NONE;                 // Chunk ID
	TArray<int32> CellIds;                      // Cell ID 목록 (배열 인덱스와 1:1)
	TArray<TArray<int32>> CellTriangles;        // Cell별 삼각형 ID 목록
	TArray<FBox> CellBounds;                    // Cell별 AABB
	bool bHasGeometry = false;                  // 해당 Chunk에 유효한 지오메트리가 있는지
	int32 MeshRevision = 0;                     // 메쉬 갱신 버전 (옵션)
};

// <추후 병목 발생시 도입 고려해볼 것들 메모>
// - (ChunkId, CellId) -> NodeIndex 조회 캐시: 인접/갱신 시 선형 탐색 최소화
// - Cell 매칭 결과 캐시: 재계산 후 Old/New CellId 매핑 유지
// - Anchor 노드 목록: BFS 시작점 스캔 비용 절감

class FRealDestructCellGraph
{
public:
	/**
	 * 격자 슬라이싱 결과로 분할 평면 리스트 생성. 
	 * Bounds는 로컬 좌표계 AABB를 넣을 것
	 */
	void BuildDivisionPlanesFromGrid(
		const FBox& Bounds, // Source static mesh's AABB
		const FIntVector& SliceCount,
		const TArray<int32>& ChunkIdByGridIndex); // Grid cell index 에 대응하는 chunk id
	
	// 특정 평면 사각 영역에 접하는 삼각형이 존재하는지 검사
	static bool HasBoundaryTrianglesOnPlane(
		const FDynamicMesh3& Mesh,
		const TArray<int32>& TriangleIds,
		const FChunkDivisionPlaneRect& Plane,
		float PlaneTolerance,
		float RectTolerance,
		TArray<FChunkBoundaryTriangle2D>& OutTriangles,
		FBox2D& OutBounds);

	// 두 노드가 동일 평면 기준으로 여전히 연결되어 있는지 검사
	static bool AreNodesConnectedByPlane(
		const FDynamicMesh3& MeshA,
		const TArray<int32>& TriangleIdsA,
		const FDynamicMesh3& MeshB,
		const TArray<int32>& TriangleIdsB,
		const FChunkDivisionPlaneRect& Plane,
		float PlaneTolerance,
		float RectTolerance);

private:
	TArray<FChunkCellNode> Nodes; // 그래프
	TArray<FChunkDivisionPlaneRect> DivisionPlanes; // 청크 분할 평면
	TArray<FChunkCellCache> ChunkCellCaches; // 각 청크의 셀, 각 셀의 삼각형 인덱스
};
