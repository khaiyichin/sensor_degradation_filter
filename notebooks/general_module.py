import numpy as np
from scipy import integrate, stats
import scipy.stats._multivariate as multivar
import matplotlib.pyplot as plt
import itertools as itt
import time
from numba import jit, njit

ZERO_APPROX = 1e-3

# Class to store a single estimate
class Estimate:
    def __init__(self, x, covar):
        self.x = np.asarray(x)
        self.covar = np.asarray(covar)

    def tolist(self):
        return [self.x.tolist(), self.covar.tolist()]

    def totuple(self):
        return tuple([
            tuple(self.x.tolist()),
            tuple(map(tuple, self.covar.tolist()))
        ])

# Class to store a single observation
class Observation:
    def __init__(self, n, t):
        self.n = n
        self.t = t
        self.complement = t - n

    def tolist(self):
        return [self.n, self.t]
    
    def totuple(self):
        return tuple(self.tolist())

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

# Define truncated bivariate normal distribution class
class TruncatedBivariateNormal:

    def __init__(self, mu, sigma):
        self.mean = mu
        self.covar = sigma
        self.truncated_norm_const = 0

    def set_norm_const(self, nc): self.truncated_norm_const = nc

    def pdf(self, x, normalized=True):
        if normalized:
            return stats.multivariate_normal.pdf(x, mean=self.mean, cov=self.covar) / self.truncated_norm_const
        else:
            return stats.multivariate_normal.pdf(x, mean=self.mean, cov=self.covar)

# Class to store combinations of parameters
class ParamCombo:

    def __init__(self, mu: list, sigma: list, curr_obs: list, past_f: list):

        self.mu = mu
        self.sigma = sigma
        self.curr_obs = curr_obs
        self.past_f = past_f
        past_est = [Estimate(*combo).totuple() for combo in itt.product(mu, sigma)]

        self.params_lst = list(
            itt.product(past_est, [(obs.totuple()) for obs in curr_obs], past_f)
        )

    def get_params(self): return self.params_lst

    def update_params(self, mu: list, sigma: list, curr_obs: list, past_f: list):

        past_est = [Estimate(*combo).totuple() for combo in itt.product(mu, sigma)]

        self.params_lst = list(
            set(
                self.params_lst + list(itt.product(past_est, [obs.totuple() for obs in curr_obs], past_f))
            )
        )

        self.mu = self.extract_unique_vectors(self.mu + mu)
        self.sigma = self.extract_unique_matrices(self.sigma + sigma)
        self.curr_obs = [
            Observation(*elem) for elem in set(
                [tuple(elem.tolist()) for elem in self.curr_obs + curr_obs]
            )
        ]

        self.past_f = list(set(self.past_f + past_f))

    def extract_unique_vectors(self, lst_arr: list):
        unique_tuples = set(tuple(arr) for arr in lst_arr)
        return [np.array(arr) for arr in unique_tuples]

    def extract_unique_matrices(self, lst_of_matrices: list):
        unique_matrices = []

        for mat in lst_of_matrices:
            # Check if the matrix is unique within tolerance
            if not any(self.are_matrices_equal(mat, unique_matrix) for unique_matrix in unique_matrices):
                unique_matrices.append(mat)

        return unique_matrices

    def are_matrices_equal(self, matrix1, matrix2, tol=1e-3):
        return np.allclose(matrix1, matrix2, atol=tol)

# Reconstruct covariance matrix
def reconstruct_cov_mat(C, diag_vals=None):
    if C.size == 3 and diag_vals is None: # C contains the diagonal and off-diagonal terms
        cov = np.diag(C[:2])
        corr = C[2]
        cov[0,1] = corr*np.sqrt(C[0]*C[1])
        cov[1,0] = cov[0,1]
    elif C.size == 2 and diag_vals is None: # C contains the diagonal terms
        cov = np.diag(C)
    elif C.size == 1 and diag_vals is not None: # C is the correlation
        cov = np.diag(diag_vals)
        cov[1,0] = C * np.sqrt(cov[0,0] * cov[1,1])
        cov[0,1] = cov[1,0]
    else:
        raise RuntimeError("Unknown input dimensions, C={0}, diag_vals={1}".format(C, diag_vals))

    return cov

# Decorator to time functions
def timing_decorator(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        print(f"{func.__name__} took {end_time - start_time} seconds to run.")
        return result
    return wrapper

#
def generate_PSD_matrix(dim):
    A = np.random.rand(dim, dim)
    return A @ A.T

# Prediction step of the filter
def prediction_step(est: Estimate, A, R):
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