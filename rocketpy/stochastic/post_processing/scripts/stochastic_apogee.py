from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from rocketpy.stochastic.post_processing.stochastic_cache import \
    SimulationCache


def compute_apogee(file_name, batch_path):
    cache = SimulationCache(
        file_name,
        batch_path,
    )
    apogee = cache.read_outputs('apogee')

    mean_apogee = float(np.nanmean(apogee, axis=0))

    return apogee, mean_apogee


def plot_apogee(batch_path, apogee, mean_apogee, save=False, show=True):
    # Histogram
    fig, ax = plt.subplots()
    ax.hist(apogee.flatten(), bins=50)
    ax.axvline(
        mean_apogee,
        color='black',
        linestyle='--',
        linewidth=2,
        label=f'Mean: {mean_apogee:.2f} m',
    )
    ax.set_title('Mean Apogee Distribution')
    ax.set_xlabel('Apogee [m]')
    ax.set_ylabel('Frequency')
    ax.legend()

    if save:
        plt.savefig(batch_path / 'mean_apogee_distribution.png')
    if show:
        plt.show()


def run(file_name, batch_path, save, show):
    apogee, mean_apogee = compute_apogee(file_name, batch_path)
    plot_apogee(batch_path, apogee, mean_apogee, save=save, show=show)


if __name__ == '__main__':
    batch_path = Path("mc_simulations/")
    file_name = 'monte_carlo_class_example'
    run(file_name, batch_path, save=True, show=True)
