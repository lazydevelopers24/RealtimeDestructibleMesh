# Realtime Destructible Mesh 릴리즈 표준 운영 절차

---

## I. Overview
릴리즈 전체 과정은 대략적으로 아래와 같습니다.
1. `develop` 브랜치에서 플러그인과 docs 갱신이 모두 완료됩니다.
2. `develop` -> `main`으로 PR을 생성 후 병합합니다.
3. `main` 브랜치에서 릴리즈할 커밋에 태그를 붙입니다. (`v1.1.0`과 같은 형식의 **semantic versioning**을 사용합니다.)
4. 태그를 붙이면 Github Actions에 의해 **Release Validation**이 수행됩니다. (태그 형식 검증 + UAT를 통한 빌드 테스트 + 모든 문서의 최신화 여부 검증)
5. **Release Validation**를 통과하면 UAT 빌드 결과물이 자동으로 업로드되고, Release가 생성됩니다.
6. Release zip 파일을 다운로드 받아 `Drive/RealtimeDestructibleMesh/Release` 폴더에 업로드합니다.
7. 액세스 권한을 **권한을 링크가 있는 모든 사용자(뷰어)**로 설정합니다.
8. 해당 파일 링크를 이용해 FAB 갱신 신청을 합니다.

---

## II. 브랜치 전략

### 1. 브랜치 구분
- 작업한 내용의 push는 기본적으로 `develop` 브랜치에서 이루어집니다. 직접 push가 가능합니다.
- 작업자가 자신이 사용하기 위한 `feat/...` 브랜치를 만들 수 있습니다.
- `main` 브랜치는 직접 push가 불가능하고, 반드시 PR을 통해서만 갱신할 수 있습니다.
- `main` 브랜치는 릴리즈 후보 역할을 담당하므로 최신 커밋은 최소한 소스 버전 엔진으로 Development Editor 빌드가 가능한 상태 유지를 권장합니다.

### 2. Pull Request
- PR의 Approval은 LazyDevelopers 계정 또는 Collaborator로 등록된 팀원 개인 계정으로만 할 수 있습니다.
- 공식적으로는 종료된 프로젝트이므로 원활한 피어리뷰가 이루어지지 않을 수 있으니, PR 작성자가 직접 Approval 및 Merge하는 것도 허용합니다.
- 하지만 PR 작성자가 직접 Approval할 경우 PR을 만들 때 **소스 버전 엔진으로 Development Editor 빌드 성공한 스크린샷** 또는 **바이너리 버전 엔진의 UAT를 통해 빌드 성공한 스크린샷**을 반드시 첨부해야 합니다.
- PR 작성자가 아닌 팀원의 Approval을 받는 경우엔 인증 스크린샷을 요구하지 않습니다.

---

## III. Release Validation
릴리즈 파일을 생성하기 위해서는 Release Validation을 통과해야 합니다.
아래 **태그 형식 검증** 항목의 규칙대로 `main` 브랜치의 커밋에 태그를 붙이면 자동으로 Release Validation이 시행되며,
Release Validation이 통과되면 릴리즈용 압축파일이 생성됩니다.

### 0. 사전 준비
Release Validation을 진행하기 위해서는 커밋을 푸쉬하는 사용자의 PC에 다음과 같은 사전 준비가 필요합니다.

1. 언리얼 엔진 5.7 바이너리 버전 경로를 시스템 환경변수 `UE_ROOT`로 등록합니다. (예시: `UE_ROOT = C:\Program Files\Epic Games\UE_5.7`)
2. 팀 Github 계정(lazydevelopers24)로 로그인해 원격 저장소로 가서 GitHub Repo → Settings → Actions → Runners → New self-hosted runner로 들어갑니다.
3. OS: Windows 선택 후 하단에 안내되어 있는 명령어대로 진행합니다.
4. config.cmd 에서 **Enter additional labels (ex. label-1,label-2):** 이란 질문이 나왔을 때 아래와 같이 runner 라벨을 붙여줍니다.
    - `self-hosted,windows,buildplugin,ue5_7,{github-username}`
    - `{github-username}`은 자신의 github username을 입력하면 됩니다. (`haedam19`, `changgeunjoe`, `WH`, `GameTithe`) 

### 1. 검증 시작
- Self-hosted runner를 실행합니다.
    - 명령 프롬프트 실행 -> runner 폴더로 이동 -> run.cmd 입력
- `main` 브랜치에 semantic versioning 규칙에 따라 `vX.Y.Z` 형식으로 태그를 달아줍니다. (Semantic versioning 각 숫자 정확한 의미 숙지 필수)
- 태그를 원격 저장소에 push하면 자동으로 Release Validation이 시작됩니다.

### 2. 검증 내용
Release Validation은 아래와 같은 내용을 검증합니다.

#### 2.1. 태그 형식 검증
- 태그 형식이 규칙과 일치하지 않으면 Release Vlidation 대상이 아닌 것으로 판단해 즉시 실패합니다.

#### 2.2. UAT 빌드 테스트
- 실행해둔 runner가 UAT BuildPlugin 명령을 실행해 플러그인 빌드를 시도합니다. 빌드가 성공적으로 수행되어야 합니다.

#### 2.3. CHANGELOG 최신화 검증
- `RealtimeDestruction/Docs/CHANGELOG.md` 파일 내에 `## v1.1.0` 같은 형식으로 붙일 태그와 일치하는 제목이 있어야 합니다.
- 태그와 일치하는 제목을 발견한 경우 제목보다 밑에 구분선(`---`)이 있는지 검사합니다. (이 구분선이 배포시 release note 끝으로 인식됩니다.)

#### 2.4. Online Documentation 최신화 검증
- `OnlineDocumentation/index.md` 파일의 `- Version:` 항목 값이 붙일 태그와 일치해야 합니다.

### 3. 검증 결과
- Release Validation이 성공하면 self-hosted runner가 `actions/checkout`으로 생성한 임시 워크스페이스(`GITHUB_WORKSPACE`)에 패키징 결과물이 생성되고, Binary 폴더와 Intermediate 폴더가 제거된 후 패키징 결과물이 압축된 zip 파일이 생성됩니다. (파일명 RealtimeDestruction_vX.Y.Z.zip)
- 원격저장소에 새 Release가 생성됩니다. CHANGELOG.md에서 버전 제목에서 시작해 처음 만나는 구분선 바로 전까지 release note로 복사되며, 빌드 테스트 결과 만들어진 zip파일이 첨부됩니다.
- 업로드가 완료되면 패키징 결과물과 zip 파일은 다시 제거됩니다.

## IV. 배포
- Github Release로 나온 파일을 FAB에 제출합니다.
- Change Log는 Github Release의 Release Note를 글자 수 제한에 맞게 요약해 적어줍니다.
