#!/usr/bin/env python3
"""Recompute chapter 3's analytic mechanisms; no robot planning or execution."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.optimize import brentq, linprog


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report-dir', type=Path, required=True)
    args = parser.parse_args()
    folder = args.report_dir.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    report = {'status': 'failed', 'scope': 'analytic single-joint model only',
              'execution_started': False, 'execution_completed': False,
              'robot_generalization_claim': False, 'random_seed': 20261126,
              'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    started = time.monotonic()
    try:
        length, first, second, omega = .50, .15, .35, .50
        required = math.asin(second / length) - math.asin(first / length)
        critical = omega * .12 / required
        report['model'] = {'arm_length_m': length, 'first_projection_max_m': first,
                           'second_projection_min_m': second, 'joint_speed_rad_s': omega,
                           'acceleration_constraint': False,
                           'between_sections': 'no geometric constraint; no stopping',
                           'joint_range_rad': [0., math.pi / 2.]}
        report['required_angle_rad'] = required
        report['critical_speed_m_s'] = critical
        report['best_fixed_configuration_margin_m'] = (first - second) / 2.
        rows = []
        for spacing in (.06, .12, .30):
            for speed in (.05, .10, critical, .20, .30):
                available = omega * spacing / speed
                feasible = available >= required - 1e-12
                margin = None
                if feasible:
                    def residual(rho):
                        return (math.asin((second + rho) / length)
                                - math.asin((first - rho) / length) - available)
                    upper = min(first, length - second)
                    margin = upper if residual(upper) <= 0. else brentq(residual, 0., upper)
                rows.append({'spacing_m': spacing, 'speed_m_s': speed,
                             'available_time_s': spacing / speed,
                             'available_angle_rad': available,
                             'feasible_speed_only': bool(feasible),
                             'max_nonnegative_margin_m': margin})
        report['sweep'] = rows
        low = next(r for r in rows if r['spacing_m'] == .12 and r['speed_m_s'] == .1)
        short = next(r for r in rows if r['spacing_m'] == .12 and r['speed_m_s'] == .2)
        long = next(r for r in rows if r['spacing_m'] == .30 and r['speed_m_s'] == .2)
        assert not short['feasible_speed_only'] and long['feasible_speed_only']
        assert abs(low['max_nonnegative_margin_m'] - .0259) < .0001
        report['equal_section_geometry_counterexample'] = {
            'short_spacing': short, 'long_spacing': long,
            'shared': 'same two section geometries, initial state and speed',
            'conclusion': 'section clearances alone do not encode connection time'}

        # Independently compare the all-pairs Lipschitz interval condition with
        # LP feasibility. Bounds between knots are affine, so piecewise-linear
        # interpolation of a feasible knot vector respects them and the speed.
        rng = np.random.default_rng(report['random_seed'])
        counts = {'feasible': 0, 'infeasible': 0, 'disagreements': 0, 'solver_errors': 0}
        for index in range(300):
            n = 2 + index % 9
            stamps = np.r_[0., np.cumsum(rng.uniform(.01, 1., n - 1))]
            lower = rng.uniform(-1.5, 1.5, n)
            upper = lower + rng.uniform(0., 1.5, n)
            rate = float(rng.uniform(.05, 2.))
            analytical = bool(np.all(lower[:, None] - upper[None, :]
                                     <= rate * np.abs(stamps[:, None] - stamps[None, :]) + 1e-10))
            matrix, bounds = [], []
            for k, dt in enumerate(np.diff(stamps)):
                row = np.zeros(n); row[k + 1], row[k] = 1., -1.
                matrix.extend((row, -row)); bounds.extend((rate * dt, rate * dt))
            solution = linprog(np.zeros(n), A_ub=matrix, b_ub=bounds,
                               bounds=list(zip(lower, upper)), method='highs')
            if solution.status not in (0, 2):
                counts['solver_errors'] += 1
            if analytical != solution.success:
                counts['disagreements'] += 1
            counts['feasible' if solution.success else 'infeasible'] += 1
        assert counts['disagreements'] == counts['solver_errors'] == 0, counts
        assert counts['feasible'] and counts['infeasible'], counts
        report['interval_condition_vs_independent_lp'] = {'cases': 300, **counts,
            'scope': 'single joint, affine interval bounds, speed only; not six-joint sufficiency'}
        speed, delay, brake, remaining = .20, .20, .25, .10
        stopping_distance = speed * delay + speed**2 / (2. * brake)
        stop_limit = math.sqrt(brake**2 * delay**2 + 2. * brake * remaining) - brake * delay
        assert abs(stopping_distance - .12) < 1e-12
        assert abs(stop_limit * delay + stop_limit**2 / (2. * brake) - remaining) < 1e-12
        report['stopping_window'] = {'speed_m_s': speed, 'delay_s': delay,
            'deceleration_m_s2': brake, 'remaining_window_m': remaining,
            'stopping_distance_m': stopping_distance, 'speed_limit_m_s': stop_limit,
            'scope': 'specified straight braking law; geometry and stop landing require separate checks'}
        with (folder / 'speed_spacing_sweep.csv').open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(9, 3.4), layout='constrained')
        spacings = np.linspace(.03, .35, 150)
        axes[0].plot(spacings, omega * spacings / required, label='Speed-only boundary')
        axes[0].scatter([.12, .30], [.20, .20], c=['#c44e52', '#55a868'], zorder=3)
        axes[0].set(xlabel='Available adjustment distance (m)', ylabel='Base speed (m/s)',
                    title='Same section geometry, different reachability')
        axes[0].legend(fontsize=8); axes[0].grid(alpha=.25)
        velocity = np.linspace(0., .3, 150)
        axes[1].plot(velocity, delay * velocity + velocity**2 / (2. * brake))
        axes[1].axhline(remaining, color='#c44e52', linestyle='--', label='Window end')
        axes[1].scatter([stop_limit], [remaining], color='#c44e52', zorder=3)
        axes[1].set(xlabel='Base speed (m/s)', ylabel='Reaction + braking distance (m)',
                    title='Retaining a stopping opportunity')
        axes[1].legend(fontsize=8); axes[1].grid(alpha=.25)
        fig.suptitle('Analytic mechanisms only; not robot execution', fontsize=10)
        fig.savefig(folder / 'constraint_mechanisms.png', dpi=180)
        fig.savefig(folder / 'constraint_mechanisms.pdf')
        plt.close(fig)
        report['status'] = 'passed'
    except Exception as error:
        report['reason'] = str(error)
    report['validation_wall_s'] = time.monotonic() - started
    (folder / 'validation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: value for key, value in report.items() if key != 'sweep'}, indent=2))
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
