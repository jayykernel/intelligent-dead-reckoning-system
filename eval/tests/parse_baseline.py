import json
import re
import sys

def parse_baseline_output(output_text):
    lines = output_text.split('\n')
    results = []
    current = {}
    in_session = False
    for line in lines:
        if line.startswith('--- Evaluating'):
            if current:
                results.append(current)
            current = {}
            # parse line like "--- Evaluating CAR: S4 ---"
            match = re.search(r'--- Evaluating (\w+): (\S+) ---', line)
            if match:
                category = match.group(1).lower()
                session = match.group(2)
                current['category'] = category
                current['session'] = session
            in_session = True
        elif line.startswith('  Outage duration:'):
            # parse outage duration and distance
            # Example: "  Outage duration: 60.0s (141.9s to 201.9s)"
            # Example: "  Distance travelled: 2.34 m [STATIONARY/IDLING <50m]"
            pass
        elif line.startswith('  Distance travelled:'):
            # Extract distance and stationary flag
            match = re.search(r'Distance travelled: ([\d.]+) m', line)
            if match:
                current['outage_dist_m'] = float(match.group(1))
            current['is_stationary'] = '[STATIONARY/IDLING <50m]' in line
        elif line.startswith('  Final position error:'):
            match = re.search(r'Final position error: ([\d.]+) m', line)
            if match:
                current['final_error_m'] = float(match.group(1))
        elif line.startswith('  Drift %:'):
            match = re.search(r'Drift %: ([\d.]+) %', line)
            if match:
                current['drift_pct'] = float(match.group(1))
            # Determine official_pass and stretch_pass
            if 'drift_pct' in current:
                current['official_pass'] = current['drift_pct'] <= 10.0 and not current.get('is_stationary', False)
                current['stretch_pass'] = current['drift_pct'] <= 2.0 and not current.get('is_stationary', False)
        elif line.startswith('  NIS GNSS Acceptance:'):
            match = re.search(r'NIS GNSS Acceptance: (\d+)/(\d+) \(([\d.]+)%\)', line)
            if match:
                current['gnss_passed'] = int(match.group(1))
                current['gnss_total'] = int(match.group(2))
                current['nis_pass_rate'] = float(match.group(3))
        elif line.startswith('--- Evaluating TWO_WHEELER:'):
            if current:
                results.append(current)
            current = {}
            match = re.search(r'--- Evaluating TWO_WHEELER: (\S+) ---', line)
            if match:
                current['category'] = 'two_wheeler'
                current['session'] = match.group(1)
                in_session = True
        elif line.startswith('--- Evaluating EDGE ENGINE'):
            if current:
                results.append(current)
            current = {}
            # Example: "--- Evaluating EDGE ENGINE (Synthetic FOG 200Hz): S1 ---"
            match = re.search(r'--- Evaluating .*?: (\S+) ---', line)
            if match:
                current['category'] = 'edge_fog'
                current['session'] = match.group(1)
                in_session = True
    if current:
        results.append(current)
    # Ensure all sessions have required fields
    for r in results:
        if 'outage_dist_m' not in r:
            r['outage_dist_m'] = 0.0
        if 'final_error_m' not in r:
            r['final_error_m'] = 0.0
        if 'drift_pct' not in r:
            r['drift_pct'] = 0.0
        if 'is_stationary' not in r:
            r['is_stationary'] = False
        if 'official_pass' not in r:
            r['official_pass'] = False
        if 'stretch_pass' not in r:
            r['stretch_pass'] = False
        if 'gnss_total' not in r:
            r['gnss_total'] = 0
        if 'gnss_passed' not in r:
            r['gnss_passed'] = 0
        if 'nis_pass_rate' not in r:
            r['nis_pass_rate'] = 0.0
    return results

if __name__ == '__main__':
    with open('eval/baseline_output.txt', 'r') as f:
        text = f.read()
    results = parse_baseline_output(text)
    with open('eval/baseline_results_phase13.json', 'w') as f:
        json.dump(results, f, indent=2)
    print(f'Parsed {len(results)} sessions')