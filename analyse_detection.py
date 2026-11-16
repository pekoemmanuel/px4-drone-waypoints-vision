#!/usr/bin/env python3
"""Analyse du journal de détection : dernier fichier detections_*.csv de ~/drone_ws/logs.

Affiche la cadence des images, la part d'images avec cible et les périodes de
détection, puis enregistre le graphique ~/drone_ws/logs/detection_vol.png.

Usage : PYTHONNOUSERSITE=1 python3 ~/drone_ws/analyse_detection.py [fichier.csv]
"""
import csv
import glob
import os
import statistics
import sys

LOG_DIR = os.path.expanduser('~/drone_ws/logs')


def charger(chemin):
    with open(chemin, newline='') as f:
        lignes = list(csv.DictReader(f))
    t = [float(r['t_s']) for r in lignes]
    detectee = [int(r['detectee']) for r in lignes]
    dx = [float(r['dx']) if r['dx'] != '' else None for r in lignes]
    part = [float(r['part']) * 100 if r['part'] != '' else None for r in lignes]
    return t, detectee, dx, part


def segments(detectee):
    """Périodes de détection consécutives : liste de (premier_indice, dernier_indice)."""
    resultat, debut = [], None
    for i, d in enumerate(detectee):
        if d and debut is None:
            debut = i
        elif not d and debut is not None:
            resultat.append((debut, i - 1))
            debut = None
    if debut is not None:
        resultat.append((debut, len(detectee) - 1))
    return resultat


def main():
    if len(sys.argv) > 1:
        chemin = sys.argv[1]
    else:
        fichiers = sorted(glob.glob(os.path.join(LOG_DIR, 'detections_*.csv')))
        if not fichiers:
            print('Aucun fichier detections_*.csv dans', LOG_DIR)
            return
        chemin = fichiers[-1]

    t, detectee, dx, part = charger(chemin)
    if len(t) < 2:
        print('Fichier presque vide :', chemin)
        return
    duree = t[-1] - t[0]
    n_det = sum(detectee)
    print(f'Fichier : {os.path.basename(chemin)}')
    print(f'{len(t)} images en {duree:.1f} s, cadence moyenne {(len(t) - 1) / duree:.1f} images/s')
    print(f'Cible vue sur {n_det} images ({100.0 * n_det / len(t):.0f} %)\n')

    segs = segments(detectee)
    if not segs:
        print('Aucune détection dans ce fichier.')
    for k, (a, b) in enumerate(segs, start=1):
        tailles = [part[i] for i in range(a, b + 1)]
        decalages = [dx[i] for i in range(a, b + 1)]
        print(f'Période {k} : de {t[a]:.1f} s à {t[b]:.1f} s ({t[b] - t[a]:.1f} s, '
              f'{b - a + 1} images) - taille {tailles[0]:.2f} % -> {tailles[-1]:.2f} % '
              f'(max {max(tailles):.2f} %), décalage horizontal moyen '
              f'{statistics.mean(decalages):+.2f}')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('\nmatplotlib absent : pas de graphique.')
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    td = [t[i] for i in range(len(t)) if detectee[i]]
    ax1.plot(td, [part[i] for i in range(len(t)) if detectee[i]], '.', markersize=4)
    ax1.set_ylabel('Taille de la cible (% de l\'image)')
    ax1.set_title('Détection de la cible rouge pendant le vol')
    ax1.grid(True, alpha=0.3)
    ax2.plot(td, [dx[i] for i in range(len(t)) if detectee[i]], '.', markersize=4, color='tab:orange')
    ax2.axhline(0, color='gray', linewidth=0.8)
    ax2.set_ylabel('Décalage horizontal\n(-1 gauche, +1 droite)')
    ax2.set_xlabel('Temps depuis le démarrage du nœud (s)')
    ax2.set_ylim(-1.05, 1.05)
    ax2.grid(True, alpha=0.3)
    for ax in (ax1, ax2):
        for a, b in segs:
            ax.axvspan(t[a], t[b], color='green', alpha=0.08)
    sortie = os.path.join(LOG_DIR, 'detection_vol.png')
    fig.tight_layout()
    fig.savefig(sortie, dpi=120)
    print(f'\nGraphique enregistré : {sortie}')


if __name__ == '__main__':
    main()
