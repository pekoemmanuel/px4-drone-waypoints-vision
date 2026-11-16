#!/usr/bin/env python3
"""Analyse d'un vol de suivi de cible : journal ~/drone_ws/logs/suivi_*.csv.

Affiche la durée de chaque phase, la situation pendant le maintien (distance à la
cible, cap, décalage, taille, altitude), la loi de taille de la cible en fonction de
la distance, puis enregistre la figure suivi_vol.png à côté du journal.

Usage : PYTHONNOUSERSITE=1 python3 ~/drone_ws/analyse_suivi.py [journal.csv]
"""
import csv
import glob
import math
import os
import statistics
import sys

import numpy as np

CIBLE_NORD = 20.0   # position de la cible dans le repère de décollage (nord, est), en m
CIBLE_EST = 0.0
COULEURS = {'ATTENTE': '#dddddd', 'DECOLLAGE': '#e8e8e8', 'RECHERCHE': '#fff2b3',
            'SUIVI': '#cfe3f7', 'MAINTIEN': '#cfeccf'}


def charger(chemin):
    with open(chemin, newline='') as f:
        lignes = list(csv.DictReader(f))
    cols = {'etat': [r['etat'] for r in lignes]}
    for nom in ('t_s', 'x', 'y', 'z', 'cap_deg', 'dx', 'part', 'vx', 'vy', 'vz', 'vlacet'):
        cols[nom] = np.array([float(r[nom]) for r in lignes])
    cols['vue'] = np.array([int(r['vue']) for r in lignes])
    cols['distance'] = np.hypot(CIBLE_NORD - cols['x'], CIBLE_EST - cols['y'])
    return cols


def phases(c):
    """Liste de (état, début, fin) en secondes."""
    resultat, debut = [], 0
    for i in range(1, len(c['etat']) + 1):
        if i == len(c['etat']) or c['etat'][i] != c['etat'][debut]:
            resultat.append((c['etat'][debut], c['t_s'][debut],
                             c['t_s'][i - 1] + 0.1 if i == len(c['etat']) else c['t_s'][i]))
            debut = i
    return resultat


def resume(c):
    print('Phases :')
    for etat, a, b in phases(c):
        print(f'  {etat:<10} de {a:5.1f} s à {b:5.1f} s ({b - a:4.1f} s)')
    idx = [i for i, e in enumerate(c['etat']) if e == 'MAINTIEN']
    if idx:
        m = np.array(idx)
        print('\nMaintien devant la cible :')
        print(f"  distance à la cible : {c['distance'][m].mean():.2f} m en moyenne "
              f"({c['distance'][m][0]:.2f} -> {c['distance'][m][-1]:.2f})")
        print(f"  cap {c['cap_deg'][m].mean():+.1f}° (écart-type {c['cap_deg'][m].std():.2f}°), "
              f"décalage moyen {c['dx'][m].mean():+.3f}, |décalage| max {abs(c['dx'][m]).max():.3f}")
        print(f"  taille de la cible {100 * c['part'][m].min():.2f} à {100 * c['part'][m].max():.2f} %, "
              f"altitude {-c['z'][m].mean():.2f} m (écart-type {c['z'][m].std():.2f})")
    suivi = [i for i, e in enumerate(c['etat']) if e == 'SUIVI']
    if suivi:
        s = np.array(suivi)
        centre = [i for i in s if abs(c['dx'][i]) < 0.1]
        if centre:
            print(f"\nSuivi : cible centrée (|décalage| < 0,1) après {c['t_s'][centre[0]] - c['t_s'][s[0]]:.1f} s")
        print(f"  vitesse d'approche commandée maximale : {np.hypot(c['vx'][s], c['vy'][s]).max():.2f} m/s")
    sel = np.array([i for i, e in enumerate(c['etat'])
                    if e in ('SUIVI', 'MAINTIEN') and c['vue'][i] and c['part'][i] > 0.0012])
    if len(sel) > 10:
        pente, ordonnee = np.polyfit(np.log(c['distance'][sel]), np.log(100 * c['part'][sel]), 1)
        a10 = math.exp(ordonnee + pente * math.log(10.0))
        print(f"\nLoi de taille ({len(sel)} points) : taille = {a10:.2f} % x (10 m / distance)^{-pente:.2f}")


def figure(c, sortie):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    t0 = next((a for e, a, b in phases(c) if e == 'DECOLLAGE'), c['t_s'][0])
    t = c['t_s'] - t0
    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    for ax in axes:
        for etat, a, b in phases(c):
            ax.axvspan(a - t0, b - t0, color=COULEURS.get(etat, '#ffffff'), alpha=0.8, lw=0)
        ax.grid(True, alpha=0.3)
    for etat, a, b in phases(c):
        axes[0].text((a + b) / 2 - t0, 1.02, etat, transform=axes[0].get_xaxis_transform(),
                     ha='center', va='bottom', fontsize=8)

    axes[0].plot(t, c['distance'], color='tab:blue')
    axes[0].set_ylabel('Distance à la cible (m)')
    axes[0].set_title('Suivi de cible en boucle fermée', pad=22)
    ax_alt = axes[0].twinx()
    ax_alt.plot(t, -c['z'], color='tab:gray', linestyle='--')
    ax_alt.set_ylabel('Altitude (m)', color='tab:gray')
    ax_alt.set_ylim(0, 6)

    axes[1].plot(t, c['cap_deg'], color='tab:red')
    axes[1].axhline(0, color='gray', linewidth=0.8)
    axes[1].set_ylabel('Cap (°)\n0° = vers la cible')

    vu = c['vue'] == 1
    axes[2].plot(t[vu], c['dx'][vu], '.', markersize=3, color='tab:orange')
    axes[2].axhline(0, color='gray', linewidth=0.8)
    axes[2].set_ylabel('Décalage horizontal\n(-1 gauche, +1 droite)')
    axes[2].set_ylim(-1.05, 1.05)

    axes[3].plot(t[vu], 100 * c['part'][vu], '.', markersize=3, color='tab:green')
    axes[3].set_ylabel("Taille de la cible\n(% de l'image)")
    axes[3].set_xlabel('Temps depuis le début du décollage (s)')

    fig.tight_layout()
    fig.savefig(sortie, dpi=120)
    print(f'\nFigure enregistrée : {sortie}')


def main():
    if len(sys.argv) > 1:
        chemin = sys.argv[1]
    else:
        journaux = sorted(glob.glob(os.path.expanduser('~/drone_ws/logs/suivi_2*.csv')))
        if not journaux:
            print('Aucun journal suivi_*.csv dans ~/drone_ws/logs')
            return
        chemin = journaux[-1]
    c = charger(chemin)
    print(f'Journal : {os.path.basename(chemin)} ({len(c["t_s"])} lignes)\n')
    resume(c)
    try:
        figure(c, os.path.join(os.path.dirname(os.path.abspath(chemin)), 'suivi_vol.png'))
    except ImportError:
        print('\nmatplotlib absent : pas de figure.')


if __name__ == '__main__':
    main()
