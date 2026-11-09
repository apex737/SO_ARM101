import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('record_check', ROOT / 'tools/check.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def has(errors, word):
    return any(word in e for e in errors)


class Records(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name) / 'demo'
        shutil.copytree(ROOT / 'examples/demo', self.root)

    def tearDown(self):
        self.temp.cleanup()

    def check(self):
        return m.check(self.root, allow_example=True)

    def mutate(self, file, fn):
        path = self.root / file
        data = m.read_page(path)
        fn(data)
        text = path.read_text(encoding='utf-8')
        a = text.index('\n', text.index('<script id="pbl-data"')) + 1
        b = text.index('</script>', a)
        body = json.dumps(data, ensure_ascii=False).replace('<', '\\u003c')
        path.write_text(text[:a] + body + '\n' + text[b:], encoding='utf-8')

    def write(self, file, text):
        path = self.root / file
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding='utf-8')

    def test_empty_and_demo(self):
        self.assertEqual(m.check(ROOT), [])
        self.assertEqual(self.check(), [])

    def test_example_flag_rejected_in_real_records(self):
        self.assertTrue(has(m.check(self.root), 'example'))

    def test_missing_block(self):
        self.mutate('project/flowchart.html', lambda d: d['links'][0].update(to='missing'))
        self.assertTrue(has(self.check(), '블록'))

    def test_missing_parameter(self):
        self.mutate('project/params.html', lambda d: d.update(items=[]))
        self.assertTrue(has(self.check(), '설정'))

    def test_missing_step(self):
        (self.root / 'project/Steps/S01.md').unlink()
        self.assertTrue(has(self.check(), 'Step'))

    def test_step_list(self):
        self.mutate('concepts.html', lambda d: d['items'][0].update(step=['S01']))
        self.assertEqual(self.check(), [])
        self.mutate('concepts.html', lambda d: d['items'][0].update(step=['S01', 'S09']))
        self.assertTrue(has(self.check(), 'S09'))

    def test_false_recovery_without_evidence(self):
        self.mutate('concepts.html', lambda d: d['items'][0].update(state='확인됨'))
        self.assertTrue(has(self.check(), '근거'))

    def test_level(self):
        record = {'date': '2026-01-01', 'prompt': '가득 찰 때 흐름 설명', 'rubric': '버림 위치', 'answer': 'practice/01-ring/answer.md', 'result': '통과', 'hints': 0, 'condition': '데모'}
        self.mutate('concepts.html', lambda d: d['items'][0].update(state='확인됨', evidence=[record]))
        self.assertTrue(has(self.check(), 'level'))
        self.mutate('concepts.html', lambda d: d['items'][0].update(level='수정'))
        self.assertEqual(self.check(), [])
        self.mutate('concepts.html', lambda d: d['items'][0].update(level='빈칸'))
        self.assertTrue(has(self.check(), 'level'))

    def test_duplicate_id(self):
        self.mutate('concepts.html', lambda d: d['items'].append(d['items'][0].copy()))
        self.assertTrue(has(self.check(), '중복'))

    def test_invalid_source(self):
        self.mutate('project/params.html', lambda d: d['items'][0].update(source='느낌'))
        self.assertTrue(has(self.check(), 'source'))

    def test_missing_decision(self):
        self.mutate('project/params.html', lambda d: d['items'][0].update(decision='D99'))
        self.assertTrue(has(self.check(), 'D99'))

    def test_invalid_step_state(self):
        path = self.root / 'project/Steps/S01.md'
        path.write_text(path.read_text(encoding='utf-8').replace('미검증', '거의 됨'), encoding='utf-8')
        self.assertTrue(has(self.check(), '현재 판정'))

    def test_project_step_table_and_goal(self):
        self.write('project/project.md', '# 프로젝트\n\n## 학습 목표\n| ID | 할 수 있게 되고 싶은 것 | 확인 방법 |\n|---|---|---|\n| L01 | 링버퍼 설명 | 실습 |\n\n## 단계 위치\n| ID | 할 일 | 파일 |\n|---|---|---|\n| S01 | 수신 | Steps/S01.md |\n| S02 | 파싱 | Steps/S02.md |\n')
        self.assertTrue(has(self.check(), 'S02'))
        self.mutate('concepts.html', lambda d: d['items'][0].update(goal='L02'))
        self.assertTrue(has(self.check(), 'L02'))

    def test_escaped_script_text(self):
        self.mutate('concepts.html', lambda d: d['items'][0].update(name='</script><img onerror=alert(1)>'))
        self.assertEqual(self.check(), [])
        self.assertIn('</script>', m.read_page(self.root / 'concepts.html')['items'][0]['name'])


if __name__ == '__main__':
    unittest.main()
