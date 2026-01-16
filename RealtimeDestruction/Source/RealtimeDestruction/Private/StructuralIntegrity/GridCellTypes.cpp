// Copyright 2025. All Rights Reserved.

#include "StructuralIntegrity/GridCellTypes.h"
#include "Components/RealtimeDestructibleMeshComponent.h"

//=============================================================================
// FDestructionShape
//=============================================================================

bool FCellDestructionShape::ContainsPoint(const FVector& Point) const
{
	switch (Type)
	{
	case EDestructionShapeType::Sphere:
		return FVector::DistSquared(Point, Center) <= Radius * Radius;

	case EDestructionShapeType::Box:
		{
			// 회전이 있는 경우
			if (!Rotation.IsNearlyZero())
			{
				// 점을 박스의 로컬 공간으로 변환
				const FVector LocalPoint = Rotation.UnrotateVector(Point - Center);

				return FMath::Abs(LocalPoint.X) <= BoxExtent.X &&
				       FMath::Abs(LocalPoint.Y) <= BoxExtent.Y &&
				       FMath::Abs(LocalPoint.Z) <= BoxExtent.Z;
			}
			else
			{
				// 회전 없음: 간단한 AABB 검사
				return FMath::Abs(Point.X - Center.X) <= BoxExtent.X &&
				       FMath::Abs(Point.Y - Center.Y) <= BoxExtent.Y &&
				       FMath::Abs(Point.Z - Center.Z) <= BoxExtent.Z;
			}
		}

	case EDestructionShapeType::Cylinder:
		{
			// XY 평면에서 거리 + Z 범위 체크
			const float DistXYSq = FMath::Square(Point.X - Center.X) +
			                       FMath::Square(Point.Y - Center.Y);

			return DistXYSq <= Radius * Radius &&
			       FMath::Abs(Point.Z - Center.Z) <= BoxExtent.Z;
		}

	case EDestructionShapeType::Line:
		{
			// 선분까지의 최단 거리
			const FVector LineDir = EndPoint - Center;
			const float LineLength = LineDir.Size();
			if (LineLength < KINDA_SMALL_NUMBER)
			{
				return false;
			}

			const FVector LineDirNorm = LineDir / LineLength;
			const FVector ToPoint = Point - Center;
			const float Projection = FVector::DotProduct(ToPoint, LineDirNorm);

			// 선분 범위 체크
			if (Projection < 0.0f || Projection > LineLength)
			{
				return false;
			}

			// 거리 체크
			const FVector ClosestPoint = Center + LineDirNorm * Projection;
			return FVector::Dist(Point, ClosestPoint) <= LineThickness;
		}
	}

	return false;
}

FCellDestructionShape FCellDestructionShape::CreateFromRequest(const FRealtimeDestructionRequest& Request)
{
	FCellDestructionShape Shape;
	Shape.Center = Request.ImpactPoint;
	Shape.Radius = Request.ShapeParams.Radius;

	switch (Request.ToolShape)
	{
	case EDestructionToolShape::Sphere:
		Shape.Type = EDestructionShapeType::Sphere;
		break;

	case EDestructionToolShape::Cylinder:
		Shape.Type = EDestructionShapeType::Line;
		Shape.EndPoint = Request.ImpactPoint - Request.ImpactNormal * Request.ShapeParams.Height;
		Shape.LineThickness = Request.ShapeParams.Radius;
		break;

	default:
		Shape.Type = EDestructionShapeType::Sphere;
		break;
	}

	return Shape;
}

//=============================================================================
// FQuantizedDestructionInput
//=============================================================================

FQuantizedDestructionInput FQuantizedDestructionInput::FromDestructionShape(const FCellDestructionShape& Shape)
{
	FQuantizedDestructionInput Result;
	Result.Type = Shape.Type;

	// cm를 mm로 변환 (정수화)
	Result.CenterMM = FIntVector(
		FMath::RoundToInt(Shape.Center.X * 10.0f),
		FMath::RoundToInt(Shape.Center.Y * 10.0f),
		FMath::RoundToInt(Shape.Center.Z * 10.0f)
	);

	Result.RadiusMM = FMath::RoundToInt(Shape.Radius * 10.0f);

	Result.BoxExtentMM = FIntVector(
		FMath::RoundToInt(Shape.BoxExtent.X * 10.0f),
		FMath::RoundToInt(Shape.BoxExtent.Y * 10.0f),
		FMath::RoundToInt(Shape.BoxExtent.Z * 10.0f)
	);

	// 각도를 0.01도 단위로 변환
	Result.RotationCentidegrees = FIntVector(
		FMath::RoundToInt(Shape.Rotation.Pitch * 100.0f),
		FMath::RoundToInt(Shape.Rotation.Yaw * 100.0f),
		FMath::RoundToInt(Shape.Rotation.Roll * 100.0f)
	);

	Result.EndPointMM = FIntVector(
		FMath::RoundToInt(Shape.EndPoint.X * 10.0f),
		FMath::RoundToInt(Shape.EndPoint.Y * 10.0f),
		FMath::RoundToInt(Shape.EndPoint.Z * 10.0f)
	);

	Result.LineThicknessMM = FMath::RoundToInt(Shape.LineThickness * 10.0f);

	return Result;
}

FCellDestructionShape FQuantizedDestructionInput::ToDestructionShape() const
{
	FCellDestructionShape Result;
	Result.Type = Type;

	// mm를 cm로 변환
	Result.Center = FVector(
		CenterMM.X * 0.1f,
		CenterMM.Y * 0.1f,
		CenterMM.Z * 0.1f
	);

	Result.Radius = RadiusMM * 0.1f;

	Result.BoxExtent = FVector(
		BoxExtentMM.X * 0.1f,
		BoxExtentMM.Y * 0.1f,
		BoxExtentMM.Z * 0.1f
	);

	Result.Rotation = FRotator(
		RotationCentidegrees.X * 0.01f,
		RotationCentidegrees.Y * 0.01f,
		RotationCentidegrees.Z * 0.01f
	);

	Result.EndPoint = FVector(
		EndPointMM.X * 0.1f,
		EndPointMM.Y * 0.1f,
		EndPointMM.Z * 0.1f
	);

	Result.LineThickness = LineThicknessMM * 0.1f;

	return Result;
}

bool FQuantizedDestructionInput::ContainsPoint(const FVector& Point) const
{
	// 양자화된 값을 사용해서 판정
	const FVector Center = FVector(CenterMM.X, CenterMM.Y, CenterMM.Z) * 0.1f;
	const float RadiusCm = RadiusMM * 0.1f;
	const FVector BoxExtentCm = FVector(BoxExtentMM.X, BoxExtentMM.Y, BoxExtentMM.Z) * 0.1f;

	switch (Type)
	{
	case EDestructionShapeType::Sphere:
		return FVector::DistSquared(Point, Center) <= RadiusCm * RadiusCm;

	case EDestructionShapeType::Box:
		{
			if (RotationCentidegrees != FIntVector::ZeroValue)
			{
				const FRotator Rot(
					RotationCentidegrees.X * 0.01f,
					RotationCentidegrees.Y * 0.01f,
					RotationCentidegrees.Z * 0.01f
				);
				const FVector LocalPoint = Rot.UnrotateVector(Point - Center);

				return FMath::Abs(LocalPoint.X) <= BoxExtentCm.X &&
				       FMath::Abs(LocalPoint.Y) <= BoxExtentCm.Y &&
				       FMath::Abs(LocalPoint.Z) <= BoxExtentCm.Z;
			}
			else
			{
				return FMath::Abs(Point.X - Center.X) <= BoxExtentCm.X &&
				       FMath::Abs(Point.Y - Center.Y) <= BoxExtentCm.Y &&
				       FMath::Abs(Point.Z - Center.Z) <= BoxExtentCm.Z;
			}
		}

	case EDestructionShapeType::Cylinder:
		{
			const float DistXYSq = FMath::Square(Point.X - Center.X) +
			                       FMath::Square(Point.Y - Center.Y);

			return DistXYSq <= RadiusCm * RadiusCm &&
			       FMath::Abs(Point.Z - Center.Z) <= BoxExtentCm.Z;
		}

	case EDestructionShapeType::Line:
		{
			const FVector EndPt = FVector(EndPointMM.X, EndPointMM.Y, EndPointMM.Z) * 0.1f;
			const float ThicknessCm = LineThicknessMM * 0.1f;

			const FVector LineDir = EndPt - Center;
			const float LineLength = LineDir.Size();
			if (LineLength < KINDA_SMALL_NUMBER)
			{
				return false;
			}

			const FVector LineDirNorm = LineDir / LineLength;
			const FVector ToPoint = Point - Center;
			const float Projection = FVector::DotProduct(ToPoint, LineDirNorm);

			if (Projection < 0.0f || Projection > LineLength)
			{
				return false;
			}

			const FVector ClosestPoint = Center + LineDirNorm * Projection;
			return FVector::Dist(Point, ClosestPoint) <= ThicknessCm;
		}
	}

	return false;
}

bool FQuantizedDestructionInput::IntersectsOBB(const FSubCellOBB& OBB) const
{
	// 양자화된 값을 cm 단위로 변환
	const FVector Center = FVector(CenterMM.X, CenterMM.Y, CenterMM.Z) * 0.1f;
	const float RadiusCm = RadiusMM * 0.1f;
	const FVector BoxExtentCm = FVector(BoxExtentMM.X, BoxExtentMM.Y, BoxExtentMM.Z) * 0.1f;

	switch (Type)
	{
	case EDestructionShapeType::Sphere:
		{
			// Sphere-OBB intersection: OBB 위의 가장 가까운 점이 구 안에 있는지 확인
			const FVector ClosestPoint = OBB.GetClosestPoint(Center);
			return FVector::DistSquared(ClosestPoint, Center) <= RadiusCm * RadiusCm;
		}

	case EDestructionShapeType::Box:
		{
			// OBB vs OBB intersection using SAT (Separating Axis Theorem)
			// 15개 축 검사: 각 박스의 3축 + 9개 에지 교차 축

			// Shape의 회전 및 축 계산
			FQuat ShapeQuat = FQuat::Identity;
			if (RotationCentidegrees != FIntVector::ZeroValue)
			{
				const FRotator ShapeRot(
					RotationCentidegrees.X * 0.01f,
					RotationCentidegrees.Y * 0.01f,
					RotationCentidegrees.Z * 0.01f
				);
				ShapeQuat = ShapeRot.Quaternion();
			}

			const FVector ShapeAxes[3] = {
				ShapeQuat.RotateVector(FVector::ForwardVector),
				ShapeQuat.RotateVector(FVector::RightVector),
				ShapeQuat.RotateVector(FVector::UpVector)
			};

			const FVector OBBAxes[3] = { OBB.AxisX, OBB.AxisY, OBB.AxisZ };

			// 두 박스 중심 간의 벡터
			const FVector D = OBB.Center - Center;

			// 15개 축에 대해 분리 검사
			auto TestAxis = [&](const FVector& Axis) -> bool
			{
				if (Axis.SizeSquared() < KINDA_SMALL_NUMBER)
				{
					return true; // 축이 너무 작으면 분리 불가로 처리
				}

				const FVector NormAxis = Axis.GetSafeNormal();

				// Shape 박스의 축 투영 반경
				float ShapeProjection = 0.0f;
				ShapeProjection += FMath::Abs(FVector::DotProduct(ShapeAxes[0], NormAxis)) * BoxExtentCm.X;
				ShapeProjection += FMath::Abs(FVector::DotProduct(ShapeAxes[1], NormAxis)) * BoxExtentCm.Y;
				ShapeProjection += FMath::Abs(FVector::DotProduct(ShapeAxes[2], NormAxis)) * BoxExtentCm.Z;

				// OBB의 축 투영 반경
				float OBBProjection = 0.0f;
				OBBProjection += FMath::Abs(FVector::DotProduct(OBBAxes[0], NormAxis)) * OBB.HalfExtents.X;
				OBBProjection += FMath::Abs(FVector::DotProduct(OBBAxes[1], NormAxis)) * OBB.HalfExtents.Y;
				OBBProjection += FMath::Abs(FVector::DotProduct(OBBAxes[2], NormAxis)) * OBB.HalfExtents.Z;

				// 중심 거리의 투영
				const float CenterDistance = FMath::Abs(FVector::DotProduct(D, NormAxis));

				// 분리 축 테스트: 중심 거리 > 투영 반경 합이면 분리됨
				return CenterDistance <= ShapeProjection + OBBProjection;
			};

			// Shape 박스의 3축 검사
			for (int32 i = 0; i < 3; ++i)
			{
				if (!TestAxis(ShapeAxes[i]))
				{
					return false;
				}
			}

			// OBB의 3축 검사
			for (int32 i = 0; i < 3; ++i)
			{
				if (!TestAxis(OBBAxes[i]))
				{
					return false;
				}
			}

			// 9개 에지 교차 축 검사 (ShapeAxis × OBBAxis)
			for (int32 i = 0; i < 3; ++i)
			{
				for (int32 j = 0; j < 3; ++j)
				{
					const FVector CrossAxis = FVector::CrossProduct(ShapeAxes[i], OBBAxes[j]);
					if (!TestAxis(CrossAxis))
					{
						return false;
					}
				}
			}

			// 모든 축에서 분리 실패 = 교차
			return true;
		}

	case EDestructionShapeType::Cylinder:
		{
			// Cylinder-OBB intersection
			// 원통은 Z축 정렬로 가정 (기존 코드와 동일한 가정)
			// OBB를 원통의 로컬 공간에서 검사

			// Z 범위 검사: OBB의 Z축 투영 범위와 원통의 Z 범위
			// OBB의 8개 꼭지점의 Z 좌표 범위 계산
			float OBBMinZ = FLT_MAX, OBBMaxZ = -FLT_MAX;
			for (int32 i = 0; i < 8; ++i)
			{
				const FVector LocalCorner(
					((i & 1) ? OBB.HalfExtents.X : -OBB.HalfExtents.X),
					((i & 2) ? OBB.HalfExtents.Y : -OBB.HalfExtents.Y),
					((i & 4) ? OBB.HalfExtents.Z : -OBB.HalfExtents.Z)
				);
				const FVector WorldCorner = OBB.LocalToWorld(LocalCorner);
				OBBMinZ = FMath::Min(OBBMinZ, WorldCorner.Z);
				OBBMaxZ = FMath::Max(OBBMaxZ, WorldCorner.Z);
			}

			// Z 범위 분리 검사
			if (OBBMaxZ < Center.Z - BoxExtentCm.Z || OBBMinZ > Center.Z + BoxExtentCm.Z)
			{
				return false;
			}

			// XY 평면에서 원과 OBB의 2D 교차 검사
			// OBB를 XY 평면에 투영한 사각형과 원의 교차
			float MinDistSq = FLT_MAX;

			// OBB의 8개 꼭지점 중 XY 평면에서 원 중심에 가장 가까운 점 찾기
			for (int32 i = 0; i < 8; ++i)
			{
				const FVector LocalCorner(
					((i & 1) ? OBB.HalfExtents.X : -OBB.HalfExtents.X),
					((i & 2) ? OBB.HalfExtents.Y : -OBB.HalfExtents.Y),
					((i & 4) ? OBB.HalfExtents.Z : -OBB.HalfExtents.Z)
				);
				const FVector WorldCorner = OBB.LocalToWorld(LocalCorner);
				const float DistSq = FMath::Square(WorldCorner.X - Center.X) + FMath::Square(WorldCorner.Y - Center.Y);
				MinDistSq = FMath::Min(MinDistSq, DistSq);
			}

			// 꼭지점 중 하나라도 원 안에 있으면 교차
			if (MinDistSq <= RadiusCm * RadiusCm)
			{
				return true;
			}

			// OBB 중심이 원 안에 있는지
			const float CenterDistSq = FMath::Square(OBB.Center.X - Center.X) + FMath::Square(OBB.Center.Y - Center.Y);
			if (CenterDistSq <= RadiusCm * RadiusCm)
			{
				return true;
			}

			// OBB의 4개 에지(XY 평면 투영)와 원의 교차
			// 보수적 근사: OBB 바운딩 원과 비교
			const float OBBRadiusXY = FMath::Sqrt(
				FMath::Square(OBB.HalfExtents.X * OBB.AxisX.X + OBB.HalfExtents.Y * OBB.AxisY.X) +
				FMath::Square(OBB.HalfExtents.X * OBB.AxisX.Y + OBB.HalfExtents.Y * OBB.AxisY.Y)
			) + FMath::Sqrt(
				FMath::Square(OBB.HalfExtents.Z * OBB.AxisZ.X) +
				FMath::Square(OBB.HalfExtents.Z * OBB.AxisZ.Y)
			);

			return CenterDistSq <= FMath::Square(RadiusCm + OBBRadiusXY);
		}

	case EDestructionShapeType::Line:
		{
			// Line-OBB intersection using Slab method
			const FVector EndPt = FVector(EndPointMM.X, EndPointMM.Y, EndPointMM.Z) * 0.1f;
			const float ThicknessCm = LineThicknessMM * 0.1f;

			const FVector LineDir = EndPt - Center;
			const float LineLength = LineDir.Size();

			if (LineLength < KINDA_SMALL_NUMBER)
			{
				// 점으로 간주 - 점이 OBB 내부에 있는지
				const FVector LocalPoint = OBB.WorldToLocal(Center);
				return FMath::Abs(LocalPoint.X) <= OBB.HalfExtents.X + ThicknessCm &&
				       FMath::Abs(LocalPoint.Y) <= OBB.HalfExtents.Y + ThicknessCm &&
				       FMath::Abs(LocalPoint.Z) <= OBB.HalfExtents.Z + ThicknessCm;
			}

			// 두께를 고려하여 OBB 확장
			const FSubCellOBB ExpandedOBB(
				OBB.Center,
				OBB.HalfExtents + FVector(ThicknessCm),
				FQuat::FindBetweenNormals(FVector::ForwardVector, OBB.AxisX)
			);
			// 더 정확하게는 OBB 축을 직접 사용
			FSubCellOBB TestOBB;
			TestOBB.Center = OBB.Center;
			TestOBB.HalfExtents = OBB.HalfExtents + FVector(ThicknessCm);
			TestOBB.AxisX = OBB.AxisX;
			TestOBB.AxisY = OBB.AxisY;
			TestOBB.AxisZ = OBB.AxisZ;

			// Slab method: 선분을 OBB의 로컬 공간으로 변환
			const FVector LocalStart = TestOBB.WorldToLocal(Center);
			const FVector LocalEnd = TestOBB.WorldToLocal(EndPt);
			const FVector LocalDir = LocalEnd - LocalStart;

			float tMin = 0.0f;
			float tMax = 1.0f;

			// 각 축에 대해 slab 교차 계산
			for (int32 Axis = 0; Axis < 3; ++Axis)
			{
				float Start, Dir, Extent;
				switch (Axis)
				{
				case 0: Start = LocalStart.X; Dir = LocalDir.X; Extent = TestOBB.HalfExtents.X; break;
				case 1: Start = LocalStart.Y; Dir = LocalDir.Y; Extent = TestOBB.HalfExtents.Y; break;
				case 2: Start = LocalStart.Z; Dir = LocalDir.Z; Extent = TestOBB.HalfExtents.Z; break;
				default: continue;
				}

				if (FMath::Abs(Dir) < KINDA_SMALL_NUMBER)
				{
					// 레이가 slab과 평행
					if (Start < -Extent || Start > Extent)
					{
						return false; // slab 밖에서 평행 = 교차 없음
					}
				}
				else
				{
					float t1 = (-Extent - Start) / Dir;
					float t2 = (Extent - Start) / Dir;

					if (t1 > t2)
					{
						Swap(t1, t2);
					}

					tMin = FMath::Max(tMin, t1);
					tMax = FMath::Min(tMax, t2);

					if (tMin > tMax)
					{
						return false; // 교차 구간 없음
					}
				}
			}

			return true; // 모든 slab과 교차
		}
	}

	return false;
}

//=============================================================================
// FGridCellCache
//=============================================================================

int32 FGridCellCache::GetAnchorCount() const
{
	int32 Count = 0;
	// 유효 셀만 순회 (희소 배열)
	for (int32 CellId : SparseIndexToCellId)
	{
		if (GetCellIsAnchor(CellId))
		{
			Count++;
		}
	}
	return Count;
}

int32 FGridCellCache::WorldPosToId(const FVector& WorldPos, const FTransform& MeshTransform) const
{
	// 월드 좌표를 로컬 좌표로 변환
	const FVector LocalPos = MeshTransform.InverseTransformPosition(WorldPos);

	// 격자 좌표 계산
	const int32 X = FMath::FloorToInt((LocalPos.X - GridOrigin.X) / CellSize.X);
	const int32 Y = FMath::FloorToInt((LocalPos.Y - GridOrigin.Y) / CellSize.Y);
	const int32 Z = FMath::FloorToInt((LocalPos.Z - GridOrigin.Z) / CellSize.Z);

	// 범위 체크
	if (X < 0 || X >= GridSize.X ||
	    Y < 0 || Y >= GridSize.Y ||
	    Z < 0 || Z >= GridSize.Z)
	{
		return INDEX_NONE;
	}

	return CoordToId(X, Y, Z);
}

FVector FGridCellCache::IdToWorldCenter(int32 CellId, const FTransform& MeshTransform) const
{
	const FVector LocalCenter = IdToLocalCenter(CellId);
	return MeshTransform.TransformPosition(LocalCenter);
}

FVector FGridCellCache::IdToLocalCenter(int32 CellId) const
{
	if (!IsValidCellId(CellId))
	{
		return FVector::ZeroVector;
	}

	const FIntVector Coord = IdToCoord(CellId);
	return FVector(
		GridOrigin.X + (Coord.X + 0.5f) * CellSize.X,
		GridOrigin.Y + (Coord.Y + 0.5f) * CellSize.Y,
		GridOrigin.Z + (Coord.Z + 0.5f) * CellSize.Z
	);
}

FVector FGridCellCache::IdToWorldMin(int32 CellId, const FTransform& MeshTransform) const
{
	const FVector LocalMin = IdToLocalMin(CellId);
	return MeshTransform.TransformPosition(LocalMin);
}

FVector FGridCellCache::IdToLocalMin(int32 CellId) const
{
	if (!IsValidCellId(CellId))
	{
		return FVector::ZeroVector;
	}

	const FIntVector Coord = IdToCoord(CellId);
	return FVector(
		GridOrigin.X + Coord.X * CellSize.X,
		GridOrigin.Y + Coord.Y * CellSize.Y,
		GridOrigin.Z + Coord.Z * CellSize.Z
	);
}

TArray<FVector> FGridCellCache::GetCellVertices(int32 CellId) const
{
	TArray<FVector> Vertices;
	Vertices.Reserve(8);

	const FVector Min = IdToLocalMin(CellId);

	// 8개 꼭지점 (비트 연산으로 각 축 오프셋 결정)
	for (int32 i = 0; i < 8; i++)
	{
		Vertices.Add(FVector(
			Min.X + ((i & 1) ? CellSize.X : 0.0f),
			Min.Y + ((i & 2) ? CellSize.Y : 0.0f),
			Min.Z + ((i & 4) ? CellSize.Z : 0.0f)
		));
	}

	return Vertices;
}

void FGridCellCache::Reset()
{
	GridSize = FIntVector::ZeroValue;
	GridOrigin = FVector::ZeroVector;
	MeshScale = FVector::OneVector;

	// 비트필드 초기화
	CellExistsBits.Empty();
	CellIsAnchorBits.Empty();

	// 희소 배열 초기화
	CellIdToSparseIndex.Empty();
	SparseIndexToCellId.Empty();
	SparseCellTriangles.Empty();
	SparseCellNeighbors.Empty();
}

bool FGridCellCache::IsValid() const
{
	if (GridSize.X <= 0 || GridSize.Y <= 0 || GridSize.Z <= 0)
	{
		return false;
	}

	const int32 TotalCells = GetTotalCellCount();
	const int32 RequiredWords = (TotalCells + 31) >> 5;  // ceil(TotalCells / 32)

	// 비트필드 크기 검증
	if (CellExistsBits.Num() != RequiredWords || CellIsAnchorBits.Num() != RequiredWords)
	{
		return false;
	}

	// 희소 배열 일관성 검증
	const int32 ValidCellCount = SparseIndexToCellId.Num();
	return SparseCellTriangles.Num() == ValidCellCount &&
	       SparseCellNeighbors.Num() == ValidCellCount &&
	       CellIdToSparseIndex.Num() == ValidCellCount;
}

TArray<int32> FGridCellCache::GetCellsInAABB(const FBox& WorldAABB, const FTransform& MeshTransform) const
{
	TArray<int32> Result;

	if (!IsValid())
	{
		return Result;
	}

	// 월드 AABB의 8개 꼭지점을 로컬 좌표로 변환하여 로컬 AABB 계산
	FBox LocalAABB(ForceInit);

	const FVector WorldCorners[8] = {
		FVector(WorldAABB.Min.X, WorldAABB.Min.Y, WorldAABB.Min.Z),
		FVector(WorldAABB.Max.X, WorldAABB.Min.Y, WorldAABB.Min.Z),
		FVector(WorldAABB.Min.X, WorldAABB.Max.Y, WorldAABB.Min.Z),
		FVector(WorldAABB.Max.X, WorldAABB.Max.Y, WorldAABB.Min.Z),
		FVector(WorldAABB.Min.X, WorldAABB.Min.Y, WorldAABB.Max.Z),
		FVector(WorldAABB.Max.X, WorldAABB.Min.Y, WorldAABB.Max.Z),
		FVector(WorldAABB.Min.X, WorldAABB.Max.Y, WorldAABB.Max.Z),
		FVector(WorldAABB.Max.X, WorldAABB.Max.Y, WorldAABB.Max.Z),
	};

	for (int32 i = 0; i < 8; ++i)
	{
		LocalAABB += MeshTransform.InverseTransformPosition(WorldCorners[i]);
	}

	// 로컬 AABB를 격자 좌표 범위로 변환
	const int32 MinX = FMath::Max(0, FMath::FloorToInt((LocalAABB.Min.X - GridOrigin.X) / CellSize.X));
	const int32 MinY = FMath::Max(0, FMath::FloorToInt((LocalAABB.Min.Y - GridOrigin.Y) / CellSize.Y));
	const int32 MinZ = FMath::Max(0, FMath::FloorToInt((LocalAABB.Min.Z - GridOrigin.Z) / CellSize.Z));

	const int32 MaxX = FMath::Min(GridSize.X - 1, FMath::FloorToInt((LocalAABB.Max.X - GridOrigin.X) / CellSize.X));
	const int32 MaxY = FMath::Min(GridSize.Y - 1, FMath::FloorToInt((LocalAABB.Max.Y - GridOrigin.Y) / CellSize.Y));
	const int32 MaxZ = FMath::Min(GridSize.Z - 1, FMath::FloorToInt((LocalAABB.Max.Z - GridOrigin.Z) / CellSize.Z));

	// 범위 내 모든 cell 수집
	Result.Reserve((MaxX - MinX + 1) * (MaxY - MinY + 1) * (MaxZ - MinZ + 1));

	for (int32 Z = MinZ; Z <= MaxZ; ++Z)
	{
		for (int32 Y = MinY; Y <= MaxY; ++Y)
		{
			for (int32 X = MinX; X <= MaxX; ++X)
			{
				const int32 CellId = CoordToId(X, Y, Z);
				if (GetCellExists(CellId))
				{
					Result.Add(CellId);
				}
			}
		}
	}

	return Result;
}
