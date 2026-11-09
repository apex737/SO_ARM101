#!/usr/bin/env python3
"""Optional AI-side record check. Python 3.9+, standard library only. No writes."""
import argparse
import json
import re
import sys
from pathlib import Path

CONCEPT_STATES = {'미확인', '이해 공백', '확인됨', '재확인', '범위 제외'}
STEP_STATES = {'미검증', '통과', '실패', '재검증 필요', '탐색 완료'}
SOURCES = {'실측', '시험', '계산', '추정', '미확인'}
LEVELS = {'', '설명', '수정', '직접 수행'}
PAGES = [('concepts.html', 'concepts'), ('project/params.html', 'params'), ('project/flowchart.html', 'flow')]


def read_page(path):
    text = path.read_text(encoding='utf-8')
    parts = re.findall(r'<script id="pbl-data" type="application/json">\s*(.*?)\s*</script>', text, re.S)
    if len(parts) != 1:
        raise ValueError('pbl-data 블록은 정확히 하나여야 합니다')
    return json.loads(parts[0])


def as_list(value):
    """A link field may be '', 'S01' or ['S01', 'S02']."""
    if value in ('', None):
        return []
    if isinstance(value, str):
        return [value]
    if isinstance(value, list) and all(isinstance(v, str) for v in value):
        return value
    raise TypeError


def step_files(root):
    folder = root / 'project/Steps'
    return {p.stem: p for p in folder.glob('*.md') if p.name != 'step.md'} if folder.is_dir() else {}


def check(root, allow_example=False):
    errors = []
    pages = {}
    for name, kind in PAGES:
        try:
            d = read_page(root / name)
            if d.get('version') != 1 or d.get('kind') != kind:
                raise ValueError('version/kind 불일치')
            for group in (['blocks', 'links', 'traces'] if kind == 'flow' else ['items']):
                if not isinstance(d.get(group), list):
                    raise ValueError(group + ' 배열 누락')
                seen = set()
                for x in d[group]:
                    if not isinstance(x, dict) or not isinstance(x.get('id'), str) or not x['id'] or x['id'] in seen:
                        raise ValueError(group + ' ID 누락/중복')
                    seen.add(x['id'])
            if d.get('example') and not allow_example:
                errors.append(name + ': 설명용 예시 데이터(example=true)가 실제 기록에 남아 있음')
            pages[kind] = d
        except (OSError, ValueError, TypeError) as e:
            errors.append(name + ': ' + str(e))
    if len(pages) != 3:
        return errors

    steps = step_files(root)
    step_text = {sid: p.read_text(encoding='utf-8') for sid, p in steps.items()}

    # Step 판정 값
    for sid, text in step_text.items():
        m = re.search(r'^- 현재 판정:\s*(.+?)\s*$', text, re.M)
        if not m or m.group(1) not in STEP_STATES:
            errors.append(sid + ': 현재 판정이 없거나 정해진 값(' + ' / '.join(sorted(STEP_STATES)) + ')이 아님')

    # project.md 단계 표와 Step 파일
    goals = set()
    project = root / 'project/project.md'
    if project.is_file():
        ptext = project.read_text(encoding='utf-8')
        goals = set(re.findall(r'^\|\s*(L\d+)\s*\|', ptext, re.M))
        sec = re.search(r'^## 단계 위치\s*$(.*?)(?=^## |\Z)', ptext, re.M | re.S)
        listed = set(re.findall(r'^\|\s*(S\d+)\s*\|', sec.group(1), re.M)) if sec else set()
        for sid in sorted(listed - set(steps)):
            errors.append('project.md: 단계 표의 ' + sid + ' 파일이 없음')
        for sid in sorted(set(steps) - listed):
            errors.append('project.md: ' + sid + '.md가 단계 표에 없음')

    def check_steps(x):
        try:
            ids = as_list(x.get('step', ''))
        except TypeError:
            errors.append(x['id'] + ': step은 문자열 또는 문자열 배열이어야 함')
            return
        for sid in ids:
            if not re.fullmatch(r'[A-Za-z0-9_-]+', sid) or sid not in steps:
                errors.append(x['id'] + ': Step 파일 없음/잘못된 ID ' + sid)

    params = {x['id'] for x in pages['params']['items']}
    for x in pages['params']['items']:
        check_steps(x)
        if x.get('source') not in SOURCES:
            errors.append(x['id'] + ': source는 ' + '/'.join(sorted(SOURCES)) + ' 중 하나여야 함')
        try:
            decisions = as_list(x.get('decision', ''))
        except TypeError:
            errors.append(x['id'] + ': decision은 문자열 또는 문자열 배열이어야 함')
            decisions = []
        for did in decisions:
            if not any(re.search(r'\b' + re.escape(did) + r'\b', t) for t in step_text.values()):
                errors.append(x['id'] + ': 결정 ' + did + '이(가) 어느 Step에도 없음')

    concepts = pages['concepts']
    if not isinstance(concepts.get('learner', ''), str):
        errors.append('concepts.html: learner는 문자열이어야 함')
    for x in concepts['items']:
        check_steps(x)
        if x.get('state') not in CONCEPT_STATES:
            errors.append(x['id'] + ': 잘못된 이해 상태')
        if x.get('state') == '확인됨' and not x.get('evidence'):
            errors.append(x['id'] + ': 확인됨에 평가 근거(evidence) 없음')
        if x.get('level', '') not in LEVELS:
            errors.append(x['id'] + ': level은 빈 값/설명/수정/직접 수행 중 하나여야 함')
        elif x.get('state') == '확인됨' and not x.get('level'):
            errors.append(x['id'] + ': 확인됨에 이해 수준(level) 없음')
        goal = x.get('goal', '')
        if goal and goal not in goals:
            errors.append(x['id'] + ': 학습 목표 ' + str(goal) + '이(가) project.md에 없음')

    flow = pages['flow']
    blocks = {x['id'] for x in flow['blocks']}
    for b in flow['blocks']:
        for p in b.get('params', []):
            if p not in params:
                errors.append(b['id'] + ': 없는 설정 ' + str(p))
    for link in flow['links']:
        if link.get('from') not in blocks or link.get('to') not in blocks:
            errors.append(link['id'] + ': 없는 블록 연결')
    return errors


if __name__ == '__main__':
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    p.add_argument('--example', action='store_true', help='설명용 예시 폴더를 검사할 때 사용')
    a = p.parse_args()
    errors = check(a.root, allow_example=a.example)
    for e in errors:
        print('오류:', e)
    print('기록 형식·연결 검사:', len(errors), '개 오류. 내용의 진실성·코드 변경·이해도는 자동 판정하지 않습니다.')
    raise SystemExit(bool(errors))
