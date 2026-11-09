# 기록 규칙 — AI가 기록할 때만 읽기

## 원본 위치
| 무엇 | 어디 |
|---|---|
| 사용자 배경 지식·답변 선호 | profile.md |
| 목표·학습 목표·제약·단계 위치 | project/project.md |
| 단계 | project/Steps/SNN.md (step.md를 복사) |
| 구조 | project/flowchart.html의 pbl-data JSON |
| 값·정책 | project/params.html의 pbl-data JSON |
| 이해 | concepts.html의 pbl-data JSON |
| 실습 목록 | practice/practice.md |
| 실제 답과 실습 | practice/NN-topic/ |
| 긴 로그·실험 결과 | project/evidence/ |
| 변경 이력 | git |

- Step의 현재 판정은 각 Step 파일에만 둔다.
  - 값은 미검증 / 통과 / 실패 / 재검증 필요 / 탐색 완료 중 하나다.
  - Step ID는 재사용하지 않는다.
- 기록 안에 이전 값 배열을 쌓지 않는다.
- 긴 로그는 Step에서 상대 경로로 참조하고, 코드 버전과 실험 조건을 함께 적는다.

## HTML 편집
- HTML 코드 전체를 읽지 않는다. pbl-data JSON 블록만 추출해 읽고 수정한다.
- 별도 JSON 복사본이나 브라우저 저장소를 만들지 않는다.
- 문자열의 `<`, `>`, `&`는 JSON 안에서 각각 `<`, `>`, `&`로 이스케이프한다.
- 특히 `</script>`를 그대로 넣지 않는다.

## JSON 형식

### 공통
- version=1, project=이름, updated=갱신 시각, example=false, kind.
- step 필드는 Step ID 문자열 하나 또는 배열(["S01","S03"])이다. 관련 없으면 빈 문자열.

### params
- 최상위: items 배열, not_applicable 문자열.
- 항목: id/name/value/unit/source/role/basis/up/down/pairs/code/step/decision/state.
- source: 값의 근거 종류. 실측/시험/계산/추정/미확인 중 하나.
- decision: Step 파일에 있는 결정 ID(D01 등).
- 채우는 법
  - 관련 없는 칸은 이유를 쓰고 비운다.
  - up/down이 맞지 않는 항목은 정책 변경의 영향으로 쓴다.
  - 판단할 수 없으면 미확인과 확인 방법을 적는다.
  - AI가 쓴 설명을 사람의 이해가 확인됐다는 뜻으로 쓰지 않는다.

### concepts
- 최상위: learner 문자열, items 배열.
- learner: 사용자 배경은 profile.md로 옮겼다. 이 칸은 "profile.md 참고" 안내만 둔다(검사 도구 호환을 위해 문자열 유지).
- 항목: id/name/goal/why/ability/code/step/state/level/evidence.
- goal: project.md의 학습 목표 ID(L01 등) 또는 빈 문자열.
- state: 미확인 / 이해 공백 / 확인됨 / 재확인 / 범위 제외.
  - 실제 오답만 이해 공백이다.
  - 중요한 AI 결정에 기대는 개념은 미확인으로 미리 기록할 수 있다. 사소한 구현마다 항목을 만들지는 않는다.
  - 확인됨은 그 조건에서 수행했다는 뜻이다. 장기 기억 인증이 아니다.
- level: 사용자가 실제로 해낸 가장 높은 이해 수준.
  | 값 | 뜻 |
  |---|---|
  | (빈 문자열) | 아직 수행 없음 |
  | 설명 | 흐름·결과를 자기 말로 설명하거나 예측함 |
  | 수정 | 기존 코드·조건·정책을 바꿔 동작시킴 |
  | 직접 수행 | 처음부터 작성·설계·진단함 |
  - 확인됨이면 level이 비어 있으면 안 된다.
  - 이해 공백이나 재확인으로 바뀌어도 이전에 도달한 level은 유지하고, evidence로 구분한다.
  - 실습 설계 5단계와의 대응은 practice/GUIDE.md에 있다.
- evidence: 현재 state를 뒷받침하는 평가 기록 배열.
  - 각 기록: date/prompt/rubric/answer(원문 또는 practice/ 파일 경로)/result/hints/condition.
  - 현재 판단에 쓰는 기록만 둔다(보통 조건별 1개). 대체된 기록은 git에 남는다.
  - 사용자 원문은 practice/에 그대로 보존한다.
  - 답을 듣기 전에 평가를 기록하지 않는다.
  - 같은 개념이라도 사용 환경이 다르면 확인 범위를 구분한다.

### flow
- 최상위: blocks/links/traces 배열.
- blocks: id/name/does/runs/when/shared/functions/params.
  - 실행 주체·주기가 없는 도메인이면 "해당 없음"으로 두고 처리 주체·조건을 적는다.
  - 한 화면이 복잡해지면 큰 블록으로 묶고 상세는 별도 자료로 연결한다.
- links: id/from/to/data/form/rate/via.
  - from/to는 block ID다.
  - 타입 대신 문서·정책·사용자 상태 등 실제로 전달되는 것을 적어도 된다.
- traces: id/title/steps/source.
  - 입력 하나가 결과에 이르는 구체적인 예다.
  - 실측과 설명용 가정을 구분한다.
  - 예외·버림·대기를 숨기지 않는다.

## 변경 보존
- 기록에는 현재 상태만 둔다. 이전 값·이전 평가·이전 구조는 git 기록으로 찾는다.
- 바꾼 이유
  - 관련 Step의 결정 기록에 쓴다.
  - 커밋 메시지에 Step ID와 한 줄 이유를 쓴다.
  - 커밋은 사용자가 허락했을 때만 한다.
- 범위 제외 항목을 지우지 않는다.
- git이 없을 때
  - 처음 한 번 git init을 제안한다.
  - 원하지 않으면 값·판정을 바꿀 때마다 project/evidence/history.md에 날짜 · ID · 이전 값 · 이유를 한 줄씩 덧붙인다.
  - 읽을 때는 필요한 ID만 검색한다.
- 목표·기준·코드·환경이 바뀌면
  - 관련 Step의 현재 판정을 재검증 필요로 바꾼다.
  - 단순 문서 오타 수정과 실제 동작 변경을 구분한다.
  - 이 작업은 AI가 한다. 자동 코드 감시가 아니다.

## 작업 경계 체크
1. Step
   - 사전 기준과 실제 근거가 있는가?
   - 실행하지 않은 것을 통과로 썼는가?
   - 판정 값이 정해진 다섯 개 중 하나인가?
2. 반영
   - 구조 변화가 flow에, 중요한 값 변화가 params에 반영됐는가?
   - 값의 source가 실제 근거와 맞는가?
3. 결정 이유를 당시 근거와 사후 추정으로 구분했는가?
4. 사용자 답이 없는 개념을 확인됨으로 만들었는가?
5. 목표·기준 변화가 관련 Step의 재검증에 반영됐는가?
6. JSON과 ID·Step·설정·흐름 연결이 유효한가? tools/check.py를 실행하거나 직접 검사한다.

## 보고
- 요약만 한다.
- 전체 기록·전체 이력은 요청받았거나 문제 진단에 필요할 때만 출력한다.
