import numpy as np
from scipy import integrate, stats
import scipy.stats._multivariate as multivar
import matplotlib.pyplot as plt

ZERO_APPROX = 1e-3

# Class to store a single estimate
class Estimate:
    def __init__(self, x, covar):
        self.x = x
        self.covar = covar

# Class to store a single observation
class Observation:
    def __init__(self, n, t):
        self.n = n
        self.t = t
        self.complement = t - n

# Class to store the joint distribution
class MultiVarJointDist(multivar.multi_rv_generic):
    """

    Based on:
    https://github.com/scipy/scipy/blob/v1.11.4/scipy/stats/_multivariate.py#L292-L787
    """

    def __init__(self, seed=None, dim=2):
        super().__init__(seed)
        self.dim = dim

    def update_binom_params(self, n, f, t):
        self.n = n
        self.t = t
        self.f = f

    def p(self, x):
        if x.ndim == 1: x = x.reshape((-1, 2))
        return x[:, 0] * self.f + (1 - x[:, 1]) * (1 - self.f)

    def update_norm_params(self, mu, covar):
        self.mu = mu
        self.covar = covar

    def compute_norm_const(self, limits_x=[ZERO_APPROX, 1-ZERO_APPROX], limits_y=[ZERO_APPROX, 1-ZERO_APPROX]):
        self.norm_const, _ = integrate.dblquad(
            lambda x, y: self.pdf(np.array([x, y]), False), *limits_y, *limits_x
        )

    def get_norm_const(self): return self.norm_const

    def pdf(self, x, normalized=True):
        out = np.exp(self._logpdf(x))
        if normalized:
            return np.divide(out, self.norm_const)
        else:
            return out

    def logpdf(self, x, normalized=True):
        out = self._logpdf(x)
        if normalized:
            return out - np.log(self.norm_const)
        else:
            return out

    def _logpdf(self, x):
        return np.log(stats.binom.pmf(self.n, self.t, self.p(x))) + np.log(stats.multivariate_normal.pdf(x, mean=self.mu, cov=self.covar))

#
def generate_PSD_matrix(dim):
    A = np.random.rand(dim, dim)
    return A @ A.T

# Prediction step of the filter
def prediction_step(est: Estimate):
    return Estimate(A @ est.x, A @ est.covar @ A.T + R)

# Create surface plot
def create_surface_plot(ax: plt.Axes, x, y, z, xlab, ylab, title):
    ax.plot_surface(x,
                    y,
                    z,
                    cmap='viridis')
    ax.set_xlabel(xlab)
    ax.set_ylabel(ylab)
    ax.grid()
    ax.set_title(title)

    return ax

# Create contour plot
def create_contour_plot(ax: plt.Axes, x, y, z, xlab, ylab, title):

    ax.contour(x,
               y,
               z,
               cmap='viridis')
    ax.set_xlabel(xlab)
    ax.set_ylabel(ylab)
    ax.grid()
    ax.set_title(title)

    return ax