#!/usr/bin/env python3
"""Analyse de répétabilité des vols par waypoints (CSV de ~/drone_ws/logs).

Pour chaque vol complet : temps de trajet et erreur d'arrivée à chaque waypoint.
Produit un résumé dans le terminal et le graphique ~/drone_ws/logs/repetabilite.png.
"""
import csv
import glob
import math
import os
import statistics

LOG_DIR = os.path.expanduser('~/drone_ws/logs')
TOLERANCE_M = 0.5


def load(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def analyse_flight(rows):
    """Retourne {waypoint: (temps_de_trajet_s, erreur_m)}, ou None si le vol est incomplet."""
    if not any(r['etat'] == 'LAND' for r in rows):
        return None
    result, t_start = {}, {}
    for r in rows:
        wp = int(r['waypoint'])
        t = float(r['t_s'])
        if r['etat'] == 'GOTO' and wp not in t_start:
            t_start[wp] = t
        elif r['etat'] == 'HOLD' and wp not in result:
            err = math.dist(
                (float(r['x']), float(r['y']), float(r['z'])),
                (float(r['cible_x']), float(r['cible_y']), float(r['cible_z'])))
            result[wp] = (t - t_start.get(wp, t), err)
    return result


def stdev(values):
    return statistics.stdev(values) if len(values) > 1 else 0.0


def main():
    flights = []
    for path in sorted(glob.glob(os.path.join(LOG_DIR, 'trajectoire_*.csv'))):
        rows = load(path)
        result = analyse_flight(rows) if rows else None
        if result:
            flights.append((os.path.basename(path), rows, result))
        else:
            print(f'Ignoré (vol incomplet) : {os.path.basename(path)}')
    if not flights:
        print('Aucun vol complet trouvé.')
        return

    waypoints = sorted({wp for _, _, res in flights for wp in res})
    print(f'\n{len(flights)} vol(s) complet(s)\n')
    print('Waypoint | erreur moyenne ± écart-type (m) | temps de trajet moyen ± écart-type (s)')
    means, sds, all_errors = [], [], []
    for wp in waypoints:
        errs = [res[wp][1] for _, _, res in flights if wp in res]
        times = [res[wp][0] for _, _, res in flights if wp in res]
        all_errors += errs
        means.append(statistics.mean(errs))
        sds.append(stdev(errs))
        print(f'{wp:>8} | {means[-1]:.2f} ± {sds[-1]:.2f}'
              f'                    | {statistics.mean(times):.1f} ± {stdev(times):.1f}')
    print(f'\nErreur globale : moyenne {statistics.mean(all_errors):.2f} m, '
          f'maximale {max(all_errors):.2f} m')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('\nmatplotlib absent. Pour le graphique : sudo apt install python3-matplotlib')
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    for i, (_, rows, _) in enumerate(flights, start=1):
        flying = [r for r in rows if r['etat'] != 'LAND']
        ax1.plot([float(r['y']) for r in flying], [float(r['x']) for r in flying],
                 alpha=0.7, label=f'Vol {i}')
    targets = []
    for r in flights[0][1]:
        t = (float(r['cible_y']), float(r['cible_x']))
        if t not in targets:
            targets.append(t)
    ax1.plot([t[0] for t in targets], [t[1] for t in targets], 'k*',
             markersize=12, label='Waypoints')
    ax1.set_title('Trajectoires superposées (vue de dessus)')
    ax1.set_xlabel('Est (m)')
    ax1.set_ylabel('Nord (m)')
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    ax2.bar([str(w) for w in waypoints], means, yerr=sds, capsize=4)
    ax2.axhline(TOLERANCE_M, color='r', linestyle='--', label='Tolérance 0,5 m')
    ax2.set_title("Erreur d'arrivée par waypoint")
    ax2.set_xlabel('Waypoint')
    ax2.set_ylabel('Erreur (m)')
    ax2.legend()

    out = os.path.join(LOG_DIR, 'repetabilite.png')
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(f'\nGraphique enregistré : {out}')


if __name__ == '__main__':
    main()
