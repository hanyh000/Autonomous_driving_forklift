import sys
from fp_pkg.rag_service import RAGService

sys.path.append('/home/dev/fp_ws/src')
rag = RAGService()

documents = [

    # ─────────────────────────────────────────
    # 상하차 시나리오 (통합 명령)
    # ─────────────────────────────────────────
    {
        'id': 'scenario_task_pallet_0',
        'text': (
            '1번 팔레트 상하차 작업 시나리오: '
            'LOAD001로 nav 이동 후 load_task pallet=0 수행. '
            '팔레트 1번은 BLUE 색상, carry_id=0에 해당한다. '
            'load_task가 실행되면 상차부터 하차까지 자동으로 이어진다. '
            '"1번 팔레트 상하차", "팔레트 1번 작업", "첫 번째 팔레트 작업해줘" 모두 이 시나리오.'
        ),
        'metadata': {'type': 'scenario', 'pallet': 0}
    },
    {
        'id': 'scenario_task_pallet_1',
        'text': (
            '2번 팔레트 상하차 작업 시나리오: '
            'LOAD001로 nav 이동 후 load_task pallet=1 수행. '
            '팔레트 2번은 RED 색상, carry_id=1에 해당한다. '
            'load_task가 실행되면 상차부터 하차까지 자동으로 이어진다. '
            '"2번 팔레트 상하차", "팔레트 2번 작업", "두 번째 팔레트 작업해줘" 모두 이 시나리오.'
        ),
        'metadata': {'type': 'scenario', 'pallet': 1}
    },
    {
        'id': 'scenario_task_pallet_2',
        'text': (
            '3번 팔레트 상하차 작업 시나리오: '
            'LOAD001로 nav 이동 후 load_task pallet=2 수행. '
            '팔레트 3번은 YELLOW 색상, carry_id=2에 해당한다. '
            'load_task가 실행되면 상차부터 하차까지 자동으로 이어진다. '
            '"3번 팔레트 상하차", "팔레트 3번 작업", "세 번째 팔레트 작업해줘" 모두 이 시나리오.'
        ),
        'metadata': {'type': 'scenario', 'pallet': 2}
    },
    {
        'id': 'scenario_task_only',
        'text': (
            '이동 없이 상하차 작업만 수행하는 경우: '
            '"작업 수행해", "상하차 작업해줘", "작업 시작해" 처럼 '
            '이동 언급이 없으면 nav 없이 load_task만 실행한다. '
            'nav step을 절대 추가하면 안 된다.'
        ),
        'metadata': {'type': 'scenario', 'action': 'task_only'}
    },

    # ─────────────────────────────────────────
    # 팔레트-색상 매핑
    # ─────────────────────────────────────────
    {
        'id': 'pallet_mapping',
        'text': (
            '팔레트 번호와 색상 매핑: '
            '팔레트 1번 = pallet=0 = BLUE(파란색). '
            '팔레트 2번 = pallet=1 = RED(빨간색). '
            '팔레트 3번 = pallet=2 = YELLOW(노란색). '
            '색상으로 팔레트 번호 역추론 가능. '
            '"파란 팔레트"=1번=pallet=0, "빨간 팔레트"=2번=pallet=1, "노란 팔레트"=3번=pallet=2.'
        ),
        'metadata': {'type': 'mapping'}
    },

    # ─────────────────────────────────────────
    # 노드 설명
    # ─────────────────────────────────────────
    {
        'id': 'node_load001',
        'text': (
            'LOAD001은 유일한 상하차 작업 노드. '
            '모든 팔레트(1번/2번/3번) 상하차 작업은 반드시 LOAD001에서 시작한다. '
            '"상차 노드", "작업 노드", "짐 싣는 곳" 모두 LOAD001을 가리킨다.'
        ),
        'metadata': {'type': 'node', 'node_id': 'LOAD001'}
    },
    {
        'id': 'node_chrg001',
        'text': (
            'CHRG001은 충전소 노드. '
            '배터리 부족 또는 충전이 필요할 때 이동하는 노드. '
            '"충전소", "충전 구역", "배터리 충전" 모두 CHRG001을 가리킨다.'
        ),
        'metadata': {'type': 'node', 'node_id': 'CHRG001'}
    },
    {
        'id': 'node_wait001',
        'text': (
            'WAIT001은 대기 노드. '
            '작업 없이 로봇이 대기할 때 이동하는 위치. '
            '"대기", "대기 위치", "기다려" 모두 WAIT001을 가리킨다.'
        ),
        'metadata': {'type': 'node', 'node_id': 'WAIT001'}
    },
    {
        'id': 'node_waypoints',
        'text': (
            '경유 노드: NODE001(경유1), NODE002(경유2), NODE003(경유3), NODE004(경유4). '
            '특정 경로를 지나야 할 때 사용하는 중간 경유 지점.'
        ),
        'metadata': {'type': 'node'}
    },

    # ─────────────────────────────────────────
    # 지게발 규칙
    # ─────────────────────────────────────────
    {
        'id': 'rule_fork_standalone',
        'text': (
            '지게발(포크) 단독 명령 규칙: '
            '"지게발 올려줘", "포크 올려", "지게발 내려줘" 처럼 '
            '이동 언급 없이 포크만 언급되면 fork 타입만 생성. '
            'nav를 절대 추가하면 안 된다.'
        ),
        'metadata': {'type': 'rule'}
    },
    {
        'id': 'rule_fork_after_nav',
        'text': (
            '이동 후 지게발 동작 규칙: '
            '"LOAD001 가서 지게발 올려" 처럼 이동과 포크가 함께 오면 '
            'nav → fork 순서로 steps 생성.'
        ),
        'metadata': {'type': 'rule'}
    },

    # ─────────────────────────────────────────
    # 복합 명령 규칙
    # ─────────────────────────────────────────
    {
        'id': 'rule_complex_order',
        'text': (
            '복합 명령 처리 규칙: '
            '"~한 후", "~하고", "~다음에" 같은 연결어가 있으면 '
            '언급된 순서대로 steps 배열에 추가한다. '
            '예: "LOAD001 이동 후 1번 팔레트 작업" → nav(LOAD001) + load_task(pallet=0).'
        ),
        'metadata': {'type': 'rule'}
    },
    {
        'id': 'rule_no_extra_nav',
        'text': (
            '불필요한 nav 추가 금지 규칙: '
            '작업 단독 명령("작업만 해줘", "상하차 작업 수행")에는 nav를 추가하지 않는다. '
            '포크 단독 명령에도 nav를 추가하지 않는다. '
            '이동이 명시된 경우에만 nav를 생성한다.'
        ),
        'metadata': {'type': 'rule'}
    },
]

rag.add_documents(documents)
print("RAG 초기화 완료")