#include "StructuralIntegrity/RealDestructCellGraph.h"

#include "DynamicMesh/DynamicMesh3.h"

namespace
{
	float Cross2D(const FVector2D& A, const FVector2D& B)
	{
		return (A.X * B.Y) - (A.Y * B.X);
	}

	float Orient2D(const FVector2D& A, const FVector2D& B, const FVector2D& C)
	{
		return Cross2D(B - A, C - A);
	}

	bool IsPointOnSegment2D(const FVector2D& A, const FVector2D& B, const FVector2D& P, float Epsilon)
	{
		return (P.X >= FMath::Min(A.X, B.X) - Epsilon) &&
			(P.X <= FMath::Max(A.X, B.X) + Epsilon) &&
			(P.Y >= FMath::Min(A.Y, B.Y) - Epsilon) &&
			(P.Y <= FMath::Max(A.Y, B.Y) + Epsilon);
	}

	bool SegmentsIntersect2D(const FVector2D& A, const FVector2D& B, const FVector2D& C, const FVector2D& D, float Epsilon)
	{
		const float O1 = Orient2D(A, B, C);
		const float O2 = Orient2D(A, B, D);
		const float O3 = Orient2D(C, D, A);
		const float O4 = Orient2D(C, D, B);

		if ((O1 * O2) < 0.0f && (O3 * O4) < 0.0f)
		{
			return true;
		}

		if (FMath::Abs(O1) <= Epsilon && IsPointOnSegment2D(A, B, C, Epsilon)) return true;
		if (FMath::Abs(O2) <= Epsilon && IsPointOnSegment2D(A, B, D, Epsilon)) return true;
		if (FMath::Abs(O3) <= Epsilon && IsPointOnSegment2D(C, D, A, Epsilon)) return true;
		if (FMath::Abs(O4) <= Epsilon && IsPointOnSegment2D(C, D, B, Epsilon)) return true;

		return false;
	}

	bool PointInTriangle2D(const FVector2D& P, const FVector2D& A, const FVector2D& B, const FVector2D& C, float Epsilon)
	{
		const float O1 = Orient2D(A, B, P);
		const float O2 = Orient2D(B, C, P);
		const float O3 = Orient2D(C, A, P);

		const bool bHasNeg = (O1 < -Epsilon) || (O2 < -Epsilon) || (O3 < -Epsilon);
		const bool bHasPos = (O1 > Epsilon) || (O2 > Epsilon) || (O3 > Epsilon);
		return !(bHasNeg && bHasPos);
	}

	bool BoundsOverlap2D(const FBox2D& A, const FBox2D& B)
	{
		if (!A.bIsValid || !B.bIsValid)
		{
			return false;
		}

		return (A.Min.X <= B.Max.X) && (A.Max.X >= B.Min.X) &&
			(A.Min.Y <= B.Max.Y) && (A.Max.Y >= B.Min.Y);
	}

	bool TrianglesIntersect2D(
		const FVector2D& A0, const FVector2D& A1, const FVector2D& A2,
		const FVector2D& B0, const FVector2D& B1, const FVector2D& B2,
		float Epsilon)
	{
		const FVector2D AEdges[3][2] = { { A0, A1 }, { A1, A2 }, { A2, A0 } };
		const FVector2D BEdges[3][2] = { { B0, B1 }, { B1, B2 }, { B2, B0 } };

		for (int32 i = 0; i < 3; ++i)
		{
			for (int32 j = 0; j < 3; ++j)
			{
				if (SegmentsIntersect2D(AEdges[i][0], AEdges[i][1], BEdges[j][0], BEdges[j][1], Epsilon))
				{
					return true;
				}
			}
		}

		if (PointInTriangle2D(A0, B0, B1, B2, Epsilon) ||
			PointInTriangle2D(A1, B0, B1, B2, Epsilon) ||
			PointInTriangle2D(A2, B0, B1, B2, Epsilon))
		{
			return true;
		}

		if (PointInTriangle2D(B0, A0, A1, A2, Epsilon) ||
			PointInTriangle2D(B1, A0, A1, A2, Epsilon) ||
			PointInTriangle2D(B2, A0, A1, A2, Epsilon))
		{
			return true;
		}

		return false;
	}
}

void FRealDestructCellGraph::BuildDivisionPlanesFromGrid(
	const FBox& Bounds,
	const FIntVector& SliceCount,
	const TArray<int32>& ChunkIdByGridIndex)
{
	DivisionPlanes.Reset();

	// Data validation - Slice Count 및 ChunkId 데이터 체크
	const int32 CountX = SliceCount.X;
	const int32 CountY = SliceCount.Y;
	const int32 CountZ = SliceCount.Z;
	if (CountX <= 0 || CountY <= 0 || CountZ <= 0 || ChunkIdByGridIndex.Num() < CountX * CountY * CountZ)
	{
		return;
	}

	// Data validation - Bounding Box 체크
	const FVector BoundsMin = Bounds.Min;
	const FVector BoundsMax = Bounds.Max;
	const FVector BoundsSize = BoundsMax - BoundsMin;
	if (BoundsSize.X <= 0.0 || BoundsSize.Y <= 0.0 || BoundsSize.Z <= 0.0)
	{
		return;
	}

	const double CellSizeX = BoundsSize.X / static_cast<double>(CountX);
	const double CellSizeY = BoundsSize.Y / static_cast<double>(CountY);
	const double CellSizeZ = BoundsSize.Z / static_cast<double>(CountZ);

	// 절단 평면의 사각형 영역 개수 계산
	const int32 EstimatedPlaneCount =
		(CountX - 1) * CountY * CountZ +
		(CountY - 1) * CountX * CountZ +
		(CountZ - 1) * CountX * CountY;
	if (EstimatedPlaneCount > 0)
	{
		DivisionPlanes.Reserve(EstimatedPlaneCount);
	}

	auto GridIndex = [CountX, CountY](int32 X, int32 Y, int32 Z)
	{
		return X + (Y * CountX) + (Z * CountX * CountY);
	};

	// X 축 경계 평면
	for (int32 X = 1; X < CountX; ++X)
	{
		const double PlaneX = BoundsMin.X + (CellSizeX * static_cast<double>(X));
		for (int32 Y = 0; Y < CountY; ++Y)
		{
			const double CenterY = BoundsMin.Y + (CellSizeY * (static_cast<double>(Y) + 0.5));
			for (int32 Z = 0; Z < CountZ; ++Z)
			{
				const double CenterZ = BoundsMin.Z + (CellSizeZ * (static_cast<double>(Z) + 0.5));

				const int32 IndexA = GridIndex(X - 1, Y, Z);
				const int32 IndexB = GridIndex(X, Y, Z);
				const int32 ChunkA = ChunkIdByGridIndex[IndexA];
				const int32 ChunkB = ChunkIdByGridIndex[IndexB];
				if (ChunkA == INDEX_NONE || ChunkB == INDEX_NONE)
				{
					continue;
				}

				FChunkDivisionPlaneRect Plane;
				Plane.PlaneOrigin = FVector(PlaneX, CenterY, CenterZ);
				Plane.PlaneNormal = FVector::ForwardVector;
				Plane.RectCenter = Plane.PlaneOrigin;
				Plane.RectAxisU = FVector::RightVector;
				Plane.RectAxisV = FVector::UpVector;
				Plane.HalfExtents = FVector2D(CellSizeY * 0.5, CellSizeZ * 0.5);
				Plane.ChunkA = ChunkA;
				Plane.ChunkB = ChunkB;
				DivisionPlanes.Add(Plane);
			}
		}
	}

	// Y 축 경계 평면
	for (int32 Y = 1; Y < CountY; ++Y)
	{
		const double PlaneY = BoundsMin.Y + (CellSizeY * static_cast<double>(Y));
		for (int32 X = 0; X < CountX; ++X)
		{
			const double CenterX = BoundsMin.X + (CellSizeX * (static_cast<double>(X) + 0.5));
			for (int32 Z = 0; Z < CountZ; ++Z)
			{
				const double CenterZ = BoundsMin.Z + (CellSizeZ * (static_cast<double>(Z) + 0.5));

				const int32 IndexA = GridIndex(X, Y - 1, Z);
				const int32 IndexB = GridIndex(X, Y, Z);
				const int32 ChunkA = ChunkIdByGridIndex[IndexA];
				const int32 ChunkB = ChunkIdByGridIndex[IndexB];
				if (ChunkA == INDEX_NONE || ChunkB == INDEX_NONE)
				{
					continue;
				}

				FChunkDivisionPlaneRect Plane;
				Plane.PlaneOrigin = FVector(CenterX, PlaneY, CenterZ);
				Plane.PlaneNormal = FVector::RightVector;
				Plane.RectCenter = Plane.PlaneOrigin;
				Plane.RectAxisU = FVector::ForwardVector;
				Plane.RectAxisV = FVector::UpVector;
				Plane.HalfExtents = FVector2D(CellSizeX * 0.5, CellSizeZ * 0.5);
				Plane.ChunkA = ChunkA;
				Plane.ChunkB = ChunkB;
				DivisionPlanes.Add(Plane);
			}
		}
	}

	// Z 축 경계 평면
	for (int32 Z = 1; Z < CountZ; ++Z)
	{
		const double PlaneZ = BoundsMin.Z + (CellSizeZ * static_cast<double>(Z));
		for (int32 X = 0; X < CountX; ++X)
		{
			const double CenterX = BoundsMin.X + (CellSizeX * (static_cast<double>(X) + 0.5));
			for (int32 Y = 0; Y < CountY; ++Y)
			{
				const double CenterY = BoundsMin.Y + (CellSizeY * (static_cast<double>(Y) + 0.5));

				const int32 IndexA = GridIndex(X, Y, Z - 1);
				const int32 IndexB = GridIndex(X, Y, Z);
				const int32 ChunkA = ChunkIdByGridIndex[IndexA];
				const int32 ChunkB = ChunkIdByGridIndex[IndexB];
				if (ChunkA == INDEX_NONE || ChunkB == INDEX_NONE)
				{
					continue;
				}

				FChunkDivisionPlaneRect Plane;
				Plane.PlaneOrigin = FVector(CenterX, CenterY, PlaneZ);
				Plane.PlaneNormal = FVector::UpVector;
				Plane.RectCenter = Plane.PlaneOrigin;
				Plane.RectAxisU = FVector::ForwardVector;
				Plane.RectAxisV = FVector::RightVector;
				Plane.HalfExtents = FVector2D(CellSizeX * 0.5, CellSizeY * 0.5);
				Plane.ChunkA = ChunkA;
				Plane.ChunkB = ChunkB;
				DivisionPlanes.Add(Plane);
			}
		}
	}
}

bool FRealDestructCellGraph::HasBoundaryTrianglesOnPlane(
	const FDynamicMesh3& Mesh,
	const TArray<int32>& TriangleIds,
	const FChunkDivisionPlaneRect& Plane,
	float PlaneTolerance,
	float RectTolerance,
	TArray<FChunkBoundaryTriangle2D>& OutTriangles,
	FBox2D& OutBounds)
{
	OutTriangles.Reset();
	OutBounds = FBox2D(ForceInit);

	if (TriangleIds.Num() == 0)
	{
		return false;
	}

	const FVector PlaneNormal = Plane.PlaneNormal.GetSafeNormal();
	const FVector AxisU = Plane.RectAxisU.GetSafeNormal();
	const FVector AxisV = Plane.RectAxisV.GetSafeNormal();
	if (PlaneNormal.IsNearlyZero() || AxisU.IsNearlyZero() || AxisV.IsNearlyZero())
	{
		return false;
	}

	const float AbsPlaneTolerance = FMath::Abs(PlaneTolerance);
	const float AbsRectTolerance = FMath::Abs(RectTolerance);
	const float MaxU = FMath::Abs(Plane.HalfExtents.X) + AbsRectTolerance;
	const float MaxV = FMath::Abs(Plane.HalfExtents.Y) + AbsRectTolerance;
	const float MinU = -MaxU;
	const float MinV = -MaxV;

	for (int32 TriId : TriangleIds)
	{
		if (!Mesh.IsTriangle(TriId))
		{
			continue;
		}

		const FIndex3i Tri = Mesh.GetTriangle(TriId);
		const int32 VertIds[3] = { Tri.A, Tri.B, Tri.C };

		bool bAllOnPlane = true;
		FVector2D UVs[3];

		for (int32 i = 0; i < 3; ++i)
		{
			const FVector3d Pos3d = Mesh.GetVertex(VertIds[i]);
			const FVector Pos(static_cast<float>(Pos3d.X), static_cast<float>(Pos3d.Y), static_cast<float>(Pos3d.Z));
			const float Dist = FVector::DotProduct(PlaneNormal, Pos - Plane.PlaneOrigin);
			if (FMath::Abs(Dist) > AbsPlaneTolerance)
			{
				bAllOnPlane = false;
				break;
			}

			const FVector Local = Pos - Plane.RectCenter;
			const float UCoord = FVector::DotProduct(Local, AxisU);
			const float VCoord = FVector::DotProduct(Local, AxisV);
			UVs[i] = FVector2D(UCoord, VCoord);
		}

		if (!bAllOnPlane)
		{
			continue;
		}

		FBox2D TriBounds(ForceInit);
		TriBounds += UVs[0];
		TriBounds += UVs[1];
		TriBounds += UVs[2];

		const bool bOverlapRect =
			(TriBounds.Min.X <= MaxU && TriBounds.Max.X >= MinU) &&
			(TriBounds.Min.Y <= MaxV && TriBounds.Max.Y >= MinV);
		if (!bOverlapRect)
		{
			continue;
		}

		FChunkBoundaryTriangle2D OutTri;
		OutTri.P0 = UVs[0];
		OutTri.P1 = UVs[1];
		OutTri.P2 = UVs[2];
		OutTri.Bounds = TriBounds;
		OutTriangles.Add(OutTri);

		if (!OutBounds.bIsValid)
		{
			OutBounds = TriBounds;
		}
		else
		{
			OutBounds.Min.X = FMath::Min(OutBounds.Min.X, TriBounds.Min.X);
			OutBounds.Min.Y = FMath::Min(OutBounds.Min.Y, TriBounds.Min.Y);
			OutBounds.Max.X = FMath::Max(OutBounds.Max.X, TriBounds.Max.X);
			OutBounds.Max.Y = FMath::Max(OutBounds.Max.Y, TriBounds.Max.Y);
		}
	}

	return OutTriangles.Num() > 0;
}

bool FRealDestructCellGraph::AreNodesConnectedByPlane(
	const FDynamicMesh3& MeshA,
	const TArray<int32>& TriangleIdsA,
	const FDynamicMesh3& MeshB,
	const TArray<int32>& TriangleIdsB,
	const FChunkDivisionPlaneRect& Plane,
	float PlaneTolerance,
	float RectTolerance)
{
	TArray<FChunkBoundaryTriangle2D> BoundaryA;
	TArray<FChunkBoundaryTriangle2D> BoundaryB;
	FBox2D BoundsA(ForceInit);
	FBox2D BoundsB(ForceInit);

	const bool bHasA = HasBoundaryTrianglesOnPlane(
		MeshA, TriangleIdsA, Plane, PlaneTolerance, RectTolerance, BoundaryA, BoundsA);
	if (!bHasA)
	{
		return false;
	}

	const bool bHasB = HasBoundaryTrianglesOnPlane(
		MeshB, TriangleIdsB, Plane, PlaneTolerance, RectTolerance, BoundaryB, BoundsB);
	if (!bHasB)
	{
		return false;
	}

	if (!BoundsOverlap2D(BoundsA, BoundsB))
	{
		return false;
	}

	const float Epsilon = FMath::Max(RectTolerance, KINDA_SMALL_NUMBER);

	for (const FChunkBoundaryTriangle2D& TriA : BoundaryA)
	{
		for (const FChunkBoundaryTriangle2D& TriB : BoundaryB)
		{
			if (!BoundsOverlap2D(TriA.Bounds, TriB.Bounds))
			{
				continue;
			}

			if (TrianglesIntersect2D(TriA.P0, TriA.P1, TriA.P2, TriB.P0, TriB.P1, TriB.P2, Epsilon))
			{
				return true;
			}
		}
	}

	return false;
}
