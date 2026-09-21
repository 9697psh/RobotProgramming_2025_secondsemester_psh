import os
import glob
import json
from collections import defaultdict
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# --- Constants ---
KOR_TO_ENG_DAY = {
    '월요일': 'Monday',
    '화요일': 'Tuesday',
    '수요일': 'Wednesday',
    '목요일': 'Thursday',
    '금요일': 'Friday',
    '토요일': 'Saturday',
    '일요일': 'Sunday',
}

def find_korean_font():
    """
    시스템에 설치된 한국어 폰트를 찾아서 설정합니다.
    없을 경우, 기본 폰트를 사용합니다.
    """
    font_name = 'NanumGothic'
    if any(font.name == font_name for font in fm.fontManager.ttflist):
        plt.rcParams['font.family'] = font_name
        print(f"✅ '{font_name}' 폰트를 찾았습니다. 그래프에 적용합니다.")
        return
    for font in fm.fontManager.ttflist:
        if 'Gothic' in font.name or 'Malgun' in font.name:
            plt.rcParams['font.family'] = font.name
            print(f"✅ '{font.name}' 폰트를 찾았습니다. 그래프에 적용합니다.")
            return
    print("⚠️ 한국어 폰트를 찾지 못했습니다. 그래프의 한글이 깨질 수 있습니다.")
    print("   - 해결 방법: 'sudo apt-get install fonts-nanum' 실행 후 다시 시도해 보세요.")


def analyze_daily_reports(report_directory):
    """
    리포트 디렉토리에서 JSON 파일들을 읽어 일별 쓰레기 데이터를 집계합니다.
    """
    print(f"요일별 리포트 분석: {report_directory}")
    json_files = glob.glob(os.path.join(report_directory, '*.json'))
    if not json_files:
        print("❌ 분석할 리포트 파일이 없습니다.")
        return None
    daily_data = []
    for file_path in json_files:
        day_kor = os.path.basename(file_path).replace('.json', '')
        # 한글 요일을 영어로 변환, 없으면 원래 이름 사용
        day_eng = KOR_TO_ENG_DAY.get(day_kor, day_kor)
        
        total_cans = 0
        total_boxes = 0
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                scan_results = data.get('results', [])
                if not scan_results and isinstance(data, list):
                    scan_results = data
                for entry in scan_results:
                    if 'final' in entry:
                        final_scan = entry.get('final', {})
                        total_cans += final_scan.get('can', 0)
                        total_boxes += final_scan.get('box', 0)
                    elif 'left' in entry and 'right' in entry:
                        left = entry.get('left', {})
                        right = entry.get('right', {})
                        total_cans += left.get('can', 0) + right.get('can', 0)
                        total_boxes += left.get('box', 0) + right.get('box', 0)

            daily_data.append({'day': day_eng, 'can': total_cans, 'box': total_boxes})
            print(f"  - {day_kor}({day_eng}): can {total_cans}개, box {total_boxes}개")
        except Exception as e:
            print(f"❗️ '{file_path}' 파일 처리 중 오류 발생: {e}")
    if not daily_data:
        print("❌ 리포트에서 유효한 데이터를 찾지 못했습니다.")
        return None
    df = pd.DataFrame(daily_data)
    df = df.set_index('day')
    return df


def create_stacked_bar_chart(df, output_filename):
    """
    데이터프레임을 사용하여 스택 막대 그래프를 생성하고 파일로 저장합니다.
    """
    if df is None or df.empty:
        print("그래프를 생성할 데이터가 없습니다.")
        return
    print("\n막대 그래프 생성")
    ax = df.plot(kind='bar', stacked=True, figsize=(10, 7), rot=0, fontsize=12)
    ax.set_title('Daily Trash Report', fontsize=18)
    ax.set_xlabel('Day of the Week', fontsize=14)
    ax.set_ylabel('Total Count', fontsize=14)
    ax.legend(title='Type', fontsize=12)
    for i, total in enumerate(df.sum(axis=1)):
        ax.text(i, total + 0.1, f'{total}', ha='center', va='bottom', fontsize=12)
    plt.tight_layout()
    plt.savefig(output_filename)
    print(f"그래프가 '{output_filename}' 파일로 저장되었습니다.")

def find_top_trash_waypoint(report_directory):
    """
    모든 리포트를 종합하여 가장 많은 쓰레기가 발견된 웨이포인트를 찾습니다.
    """
    print(f"\n 최다 쓰레기 지점 분석 중: {report_directory}")
    json_files = glob.glob(os.path.join(report_directory, '*.json'))
    if not json_files:
        return
    waypoint_counts = defaultdict(int)
    for file_path in json_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                scan_results = data.get('results', [])
                if not scan_results and isinstance(data, list):
                    scan_results = data
                for entry in scan_results:
                    waypoint = entry.get('waypoint')
                    if not waypoint:
                        continue
                    total_trash = 0
                    if 'final' in entry:
                        final_scan = entry.get('final', {})
                        total_trash = final_scan.get('can', 0) + final_scan.get('box', 0)
                    elif 'left' in entry and 'right' in entry:
                        left = entry.get('left', {})
                        right = entry.get('right', {})
                        total_trash = left.get('can', 0) + right.get('can', 0) + \
                                      left.get('box', 0) + right.get('box', 0)
                    waypoint_counts[waypoint] += total_trash
        except Exception as e:
            print(f"❗️ '{file_path}' 파일 처리 중 오류 발생: {e}")
    if not waypoint_counts:
        print("❌ 웨이포인트별 데이터를 찾을 수 없습니다.")
        return
    top_waypoint = max(waypoint_counts, key=waypoint_counts.get)
    max_trash_count = waypoint_counts[top_waypoint]
    print("\n" + "="*50)
    print("전체 기간 최다 무단투기 지점 분석 결과")
    print(f"  - 최다 발생 지점: 웨이포인트 {top_waypoint}")
    print(f"  - 발견된 총 쓰레기 수: {max_trash_count}개")
    print("="*50)


if __name__ == '__main__':
    report_dir = os.path.join(os.path.expanduser('~'), 'ros2_project_ws', 'report')
    output_file = 'daily_trash_report.png'
    find_korean_font()
    report_df = analyze_daily_reports(report_dir)
    create_stacked_bar_chart(report_df, output_file)
    find_top_trash_waypoint(report_dir)
