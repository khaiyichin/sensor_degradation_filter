try:
    from notebooks.general_module import timing_decorator, \
                            reconstruct_cov_mat, \
                            TruncatedBivariateNormal, \
                            MultiVarJointDist, \
                            Estimate, \
                            Observation, \
                            ZERO_APPROX
except:
    try:
        from general_module import timing_decorator, \
                            reconstruct_cov_mat, \
                            TruncatedBivariateNormal, \
                            MultiVarJointDist, \
                            Estimate, \
                            Observation, \
                            ZERO_APPROX
    except:
        pass
                            
import numpy as np
from scipy import integrate, optimize, stats
from numba import jit

# Wrap the minimize with a timer function
timed_minimize = timing_decorator(optimize.minimize)
timed_dblquad = timing_decorator(integrate.dblquad)

@jit(nopython=True)
def trunc_bivar_normal_pdf_numba(cls_instance: TruncatedBivariateNormal, x):
    return cls_instance.pdf(x, False)

@jit(nopython=True)
def multivar_joint_dist_pdf_numba(cls_instance: MultiVarJointDist, x):
        return np.exp(np.log(
            stats.binom.pmf(cls_instance.n, cls_instance.t, cls_instance.p(x))
        ) + np.log(
            stats.multivariate_normal.pdf(x, mean=cls_instance.mu, cov=cls_instance.covar)
        ))

class SensorFilter1D:

    class MultiVarJointDist1D:
        """Internal 1-D joint distribution.
        """
        
        def update_binom_params(self, n, f, t):
            self.n = n
            self.t = t
            self.f = f

        def p(self, x):
            return x * self.f + (1 - x) * (1 - self.f)

        def update_norm_params(self, mu, sigma_sq):
            self.mu = float(mu)
            self.sigma_sq = float(sigma_sq)

        def normalize(self):
            self.norm_const, _ = integrate.quad(
                lambda x: self.pdf(x), 0.5, 1.0
            )

        def pdf(self, x):
            """Compute the density value.

            Note: the PDF is not normalized.
            """
            return np.exp(
                self.logpdf(x)
            )

        def logpdf(self, x):
            if type(x) == np.ndarray and len(x) > 1:
                y = np.asarray([*map(float, x)])
            else:
                y = float(x)

            # Compute truncated normal truncation points
            a, b = (0.5 - self.mu) / np.sqrt(self.sigma_sq), (1.0 - self.mu) / np.sqrt(self.sigma_sq)

            return np.log(
                stats.binom.pmf(self.n, self.t, self.p(y))
            ) + np.log(
                stats.truncnorm.pdf(y, a, b, loc=self.mu, scale=np.sqrt(self.sigma_sq))
            )

        def logpdf_binom(self, x):
            if type(x) == np.ndarray and len(x) > 1:
                y = np.asarray([*map(float, x)])
            else:
                y = float(x)
            return np.log(
                stats.binom.pmf(self.n, self.t, self.p(y))
            )

        def logpdf_truncnorm(self,x):
            if type(x) == np.ndarray and len(x) > 1:
                y = np.asarray([*map(float, x)])
            else:
                y = float(x)

            # Compute truncated normal truncation points
            a, b = (0.5 - self.mu) / np.sqrt(self.sigma_sq), (1.0 - self.mu) / np.sqrt(self.sigma_sq)

            return np.log(
                stats.truncnorm.pdf(y, a, b, loc=self.mu, scale=np.sqrt(self.sigma_sq))
            )

    class TruncatedNormal1D:
        """Internal 1-D truncated normal distribution.
        """
        def __init__(self, mu=0, sigma_sq=1, limit=[0.5, 1.0]):
            self.mean = float(mu)
            self.covar = float(sigma_sq)
            self.truncated_norm_const = 0
            self.limit = [*map(float, limit)]

        def update_params(self, mu, sigma_sq):
            self.mean = float(mu)
            self.covar = float(sigma_sq)

        def pdf(self, x, normalized=True):
            if type(x) == np.ndarray and len(x) > 1:
                y = np.asarray([*map(float, x)])
            else:
                y = float(x)

            # Compute truncated normal truncation points
            a, b = (0.5 - self.mean) / np.sqrt(self.covar), (1.0 - self.mean) / np.sqrt(self.covar)

            return stats.truncnorm.pdf(
                y, a, b, loc=self.mean, scale=np.sqrt(self.covar)
            ) if normalized else stats.norm.pdf(
                y, self.mean, np.sqrt(self.covar)
            )


    def __init__(self,
                 a: float,
                 b: float,
                 r: float,
                 limit = [0.5+ZERO_APPROX, 1.0-ZERO_APPROX] # the sensor accuracies cannot be exactly 1.0 or 0.5 for numerical reasons
        ):

        self.act_joint_dist = self.MultiVarJointDist1D()
        self.est_posterior = self.TruncatedNormal1D(mu=0.5, sigma_sq=1)
        self.a = a
        self.b = b
        self.r = r
        self.limit = limit

    def degrade_quality(self, sensor_quality):
        sensor_acc = stats.norm.rvs(loc=self.a * sensor_quality[0] + self.b, scale=np.sqrt(self.r))
        if sensor_acc <= 0.5: sensor_acc = self.limit[0]
        elif sensor_acc >= 1.0: sensor_acc = self.limit[1]
        return np.asarray([[sensor_acc], [sensor_acc]])

    def neg_joint_dist_pdf(self, x):
        return -self.act_joint_dist.pdf(x)

    def kl_div(self, variance, surr_dist: TruncatedNormal1D, mu):
        surr_dist.update_params(mu, variance)

        divergence, _ = integrate.quad(
            lambda x: surr_dist.pdf(x) * (
                np.log(surr_dist.pdf(x)) - np.log(self.act_joint_dist.pdf(x)/self.act_joint_dist.norm_const)
            ), *self.limit
        )

        return divergence
    
    def neg_ELBO(self, variance, surr_dist: TruncatedNormal1D, mu):
        surr_dist.update_params(mu, variance)

        elbo, _ = integrate.quad(
            lambda x: surr_dist.pdf(x) * (
                np.log(self.act_joint_dist.pdf(x)/surr_dist.pdf(x))
            ), *self.limit
        )

        return -elbo

    def grad_neg_ELBO(self, variance, surr_dist: TruncatedNormal1D, mu):
        surr_dist.update_params(mu, variance)

        inv_variance = 1/variance

        grad_term = lambda x: inv_variance**2 * (x-mu)**2

        factor = lambda x: np.log(self.act_joint_dist.pdf(x)/surr_dist.pdf(x)) - 1
        term_1 = grad_term
        term_2 = -integrate.quad(
            lambda x: surr_dist.pdf(x) * grad_term(x), *self.limit
        )[0]

        grad_elbo = integrate.quad(
            lambda x: surr_dist.pdf(x) * factor(x) * (term_1(x) + term_2), *self.limit
        )[0] / 2

        return -grad_elbo

    def _predict(self, est: Estimate, elapsed_duration):
        a = self.a ** elapsed_duration
        b = self.b * elapsed_duration
        r = self.r * elapsed_duration
        return Estimate(a * est.x + b, a**2 * est.covar + r)

    def _update(self, prediction: Estimate, obs: Observation, past_f):

        # Update joint distribution values
        self.act_joint_dist.update_binom_params(obs.n, past_f, obs.t)
        self.act_joint_dist.update_norm_params(prediction.x, prediction.covar)

        # Find MAP estimate for the mean
        updated_mean_result = optimize.minimize(
            self.neg_joint_dist_pdf,
            prediction.x,
            # method="L-BFGS-B",
            # bounds=[self.limit]
            method="SLSQP",
            bounds=[self.limit]
        )

        if not updated_mean_result.success:
            print(updated_mean_result)
            print("limit={0}".format(self.limit))
            print("obs n={0}, t={1}, past_f={2}, prediction.x={3}, prediction.covar={4}".format(obs.n, obs.t, past_f, prediction.x, prediction.covar))
            print("joint dist n={0}, t={1}, f={2}, mean={3}, covar={4}".format(self.act_joint_dist.n, self.act_joint_dist.t, self.act_joint_dist.f, self.act_joint_dist.mu, self.act_joint_dist.sigma_sq))
            raise RuntimeWarning("MAP estimation is unsuccessful")

        # Find the variance
        surrogate_dist = self.TruncatedNormal1D(self.est_posterior.mean, self.est_posterior.covar)

        # updated_var_result = timed_minimize(
        updated_var_result = optimize.minimize(
            self.neg_ELBO,
            # self.neg_ELBO_manual,
            prediction.covar,
            # jac=self.grad_neg_ELBO,
            args=(surrogate_dist, updated_mean_result.x),
            method="SLSQP",
            bounds=[(ZERO_APPROX, np.inf)]
        )

        if not updated_var_result.success:
            print("##### Variance estimation is unsuccessful #####")
            print("limits:", self.limit)
            print("prediction.x", prediction.x)
            print("prediction.covar", prediction.covar)
            print("updated_mean_result:\n",updated_mean_result)
            print("updated_var_result:\n", updated_var_result)
            print("surrogate mean={0}, covar={1}, truncated_norm_const={2}, limit={3}".format(surrogate_dist.mean,surrogate_dist.covar,surrogate_dist.truncated_norm_const,surrogate_dist.limit))
            print("joint dist n={0}, t={1}, f={2}, mean={3}, covar={4}".format(self.act_joint_dist.n, self.act_joint_dist.t, self.act_joint_dist.f, self.act_joint_dist.mu, self.act_joint_dist.sigma_sq))
            print("##### ##### #####")

        self.est_posterior.mean = surrogate_dist.mean
        self.est_posterior.covar = surrogate_dist.covar

        return (
            updated_mean_result,
            updated_var_result
        )

    def estimate(self, est: Estimate, obs: Observation, past_f, elapsed_duration):

        est.x = est.x[0]
        est.covar = est.covar[0]

        prediction = self._predict(est, elapsed_duration)
        mean_result, var_result = self._update(prediction, obs, past_f)

        output = (
            Estimate(
                np.ones((2,1)) * mean_result.x,
                np.ones((2,1)) * var_result.x
            ),
            (mean_result, var_result) # optimization output
        )

        return output


class SensorFilter1DAlpha:
    """Use the collective perception equation and modify f to use social estimate to update sensor accuracy
    """

    def __init__(
            self,
            limit
        ):
        self.limit = limit

    def _update(self, obs: Observation, soc_est):

        num = (obs.n / obs.t + soc_est - 1.0)
        denom = (2*soc_est - 1)

        updated_estimate = num / denom

        if updated_estimate >= 1.0: return 9.99999e-1
        elif updated_estimate <= 0.0: return 1e-6
        else:
            return updated_estimate

    def estimate(self, est: Estimate, obs: Observation, soc_est):

        # Not using est parameter

        mean_result = self._update(obs, soc_est)

        output = (
            Estimate(
                np.ones((2,1)) * mean_result,
                np.zeros((2,1))
            ),
            (mean_result)
        )

        return output

class SensorFilter2D:
    # @todo: need to implement B matrix

    def __init__(self,
                 A: np.ndarray,
                 R: np.ndarray,
                 limit_x = [ZERO_APPROX, 1.0],
                 limit_y = [ZERO_APPROX, 1.0]
        ):

        self.act_joint_dist = MultiVarJointDist(dim=2)
        self.est_posterior = TruncatedBivariateNormal(mu=[0.5, 0.5], sigma=np.eye(2)) # our approximation of the actual posterior
        self.A = A # nominal sensor degradation model (state transition matrix)
        self.R = R # nominal covariance for degradation model
        self.limit_x = limit_x
        self.limit_y = limit_y

    def degrade_quality(self, sensor_quality):
        return self.A @ sensor_quality
        # return stats.norm.rvs(loc=self.A @ sensor_quality + self.B, scale=self.R) # not tested

    def neg_joint_dist_pdf(self, x):
        """Negated density value of the joint density.

        Can be used as the minimization cost to find the MAP estimate.

        Args:
            x: Support of the joint distribution.
        """
        return -multivar_joint_dist_pdf_numba(self.act_joint_dist, x) # the normalization constant isn't computed nor is it required

    def neg_ELBO(
            self,
            C: np.ndarray,
            surr_dist: TruncatedBivariateNormal,
            mu,
            diag_vals=None
        ):
        """Negated evidence lower bound.

        Used as a minimization cost to estimate the sensor values.
        This is written with a 2-D (2 sensor value) problem in mind.
        """

        cov = reconstruct_cov_mat(C, diag_vals)
        surr_dist.update_params(mu, cov)

        print("debug corr=", C, "diag_vals=", diag_vals, "cov=", cov)

        surrogate_norm_const, _ = integrate.dblquad(
            lambda x, y: trunc_bivar_normal_pdf_numba(
                surr_dist,
                np.array([x, y])
            ), *self.limit_y, *self.limit_x
        )

        ln_expectation, _ = integrate.dblquad(
            lambda x, y: trunc_bivar_normal_pdf_numba(surr_dist, np.array([x, y])) * 
                        (
                            np.log(multivar_joint_dist_pdf_numba(self.act_joint_dist, np.array([x, y]))) -
                            np.log(trunc_bivar_normal_pdf_numba(surr_dist, np.array([x, y]))) +
                            np.log(surrogate_norm_const)
                        ), *self.limit_y, *self.limit_x
        )
        return -(ln_expectation)/surrogate_norm_const

    def grad_neg_ELBO(
            self,
            C: np.ndarray,
            surr_dist: TruncatedBivariateNormal,
            mu,
            diag_vals=None
        ):
        """Jacobian of the negative ELBO.
        """
        cov = reconstruct_cov_mat(C, diag_vals)
        surr_dist.update_params(mu, cov)

        surrogate_norm_const, _ = integrate.dblquad(
            lambda x, y: trunc_bivar_normal_pdf_numba(
                surr_dist,
                np.array([x, y])
            ), *self.limit_y, *self.limit_x
        )

        delta = lambda x, y: np.reshape(np.array([x, y]) - mu, (2,1)) # dim = 2x1
        
        inv_cov = np.linalg.inv(cov)

        inv_norm_const = 1/surrogate_norm_const

        # Define the term that arises from the gradient of the surrogate distribution (the surrogate distribution has been factored out)
        grad_surr_dist_term_00 = lambda x, y: np.squeeze(inv_cov[[0], :] @ delta(x,y) @ delta(x,y).T @ inv_cov[:, [0]])
        grad_surr_dist_term_01 = lambda x, y: np.squeeze(inv_cov[[0], :] @ delta(x,y) @ delta(x,y).T @ inv_cov[:, [1]])
        grad_surr_dist_term_11 = lambda x, y: np.squeeze(inv_cov[[1], :] @ delta(x,y) @ delta(x,y).T @ inv_cov[:, [1]])
        grad_surr_dist_term_arr = [
            grad_surr_dist_term_00,
            grad_surr_dist_term_01,
            grad_surr_dist_term_11
        ]

        output = np.zeros(C.size)

        if C.size == 2: indices = [[0,0], [1,1]]
        elif C.size == 1: indices = [[0,1]]
        else: indices = [[0,0], [1,1], [0,1]]

        # Only iterate through upper triangular coefficients (because matrix is symmetric)
        for ind, (i,j) in enumerate(indices):

            factor = lambda x, y: \
                    np.log(multivar_joint_dist_pdf_numba(self.act_joint_dist, np.array([x, y]))) + \
                    0.5 * delta(x, y).T @ inv_cov @ delta(x, y) - 1 + \
                    np.log(surrogate_norm_const)

            term_1 = grad_surr_dist_term_arr[i+j]


            term_2 = -inv_norm_const * integrate.dblquad(
                lambda x, y: trunc_bivar_normal_pdf_numba(surr_dist, np.array([x, y])) *
                    grad_surr_dist_term_arr[i+j](x, y), *self.limit_y, *self.limit_x
            )[0]
            
            output[ind] = -0.5 * inv_norm_const *integrate.dblquad(
                lambda x, y: trunc_bivar_normal_pdf_numba(surr_dist, np.array([x, y])) * \
                    factor(x,y) * (term_1(x,y) + term_2), *self.limit_y, *self.limit_x
            )[0]

        return output

    def pos_def_constraint(self, C, diag_vals=None):
        """Constraint function to check for positive definiteness of covariance matrix.
        """
        cov = reconstruct_cov_mat(C, diag_vals)
        
        return (np.linalg.eigvals(cov) - np.array([1e-3, 1e-3])).ravel()
    
    def _predict(self, est: Estimate, elapsed_duration):
        A = self.A ** elapsed_duration
        return Estimate(A @ est.x, A @ est.covar @ A.T + self.R)

    def _update(self, prediction: Estimate, obs: Observation, past_f):

        # Update joint distribution values
        self.act_joint_dist.update_binom_params(obs.n, past_f, obs.t)
        self.act_joint_dist.update_norm_params(prediction.x, prediction.covar)
        # self.act_joint_dist.compute_norm_const() # this isn't required for the ELBO computation

        # Find MAP estimate for the mean
        updated_mean_result = optimize.minimize(
            self.neg_joint_dist_pdf,
            prediction.x,
            method="SLSQP",
            bounds=[self.limit_x, self.limit_y]
        )

        if not updated_mean_result.success:
            print(updated_mean_result)
            raise RuntimeWarning("MAP estimation is unsuccessful")

        # Find diagonal values of the covariance matrix
        print("debug: initial covariance value", [prediction.covar[0,0], prediction.covar[1,1]])

        surrogate_dist = TruncatedBivariateNormal()

        updated_cov_diag_result = timed_minimize(
            self.neg_ELBO,
            [prediction.covar[0,0], prediction.covar[1,1]],
            jac=self.grad_neg_ELBO,
            args=(surrogate_dist, updated_mean_result.x, None),
            method="SLSQP",
            constraints={"type": "ineq", "fun": self.pos_def_constraint},
            bounds=[*[(ZERO_APPROX, np.inf) for _ in range(2)]], # 2 variables: c_11, c_22
            options={"eps": 1e-3, "disp": True}
        )
        
        print(updated_cov_diag_result)

        if not updated_cov_diag_result.success:
            raise RuntimeWarning("Covariance diagonal estimation is unsuccessful")

        # Find correlation (off-diagonal values) of the correlation matrix
        print("debug: initial correlation value", prediction.covar[1,0] / np.sqrt(prediction.covar[0,0] * prediction.covar[1,1]))

        updated_corr_result = timed_minimize(
            self.neg_ELBO,
            [prediction.covar[1,0] / np.sqrt(prediction.covar[0,0] * prediction.covar[1,1])],
            jac=self.grad_neg_ELBO,
            bounds=[(-(1 - 1e-6), 1 - 1e-6)],
            args=(surrogate_dist, updated_mean_result.x, updated_cov_diag_result.x),
            method="SLSQP",
            constraints={
                "type": "ineq",
                "fun": self.pos_def_constraint,
                "args": (updated_cov_diag_result.x,)
            },
        )

        print(updated_corr_result)

        if not updated_corr_result.success:
            raise RuntimeWarning("Covariance correlation estimation is unsuccessful")

        return (
            updated_mean_result,
            updated_cov_diag_result,
            updated_corr_result
        )

    def estimate(self, est: Estimate, obs: Observation, past_f, elapsed_duration):

        prediction = self._predict(est, elapsed_duration)

        prediction.x = prediction.x.squeeze()
        mean_result, cov_diag_result, cov_corr_result = self._update(prediction, obs, past_f)

        output = (
            Estimate(
                mean_result.x.reshape((2,-1)),
                reconstruct_cov_mat(cov_corr_result.x, cov_diag_result.x)
            ),
            (mean_result, cov_corr_result, cov_diag_result) # optimization output
        )

        return output



class Environment:
    
    def __init__(self, fill_ratio):
        self.dist = stats.bernoulli(fill_ratio)

    def get_occurrences(self, n):
        return self.dist.rvs(size=n)

class MinimalisticCollectivePerception:

    def __init__(self):
        pass

    def make_observation(self, occurrence: int, sensor_b: float, sensor_w: float):
        """Make flawed observation based on occurrence.
        """
        if occurrence == 1: # black
            return stats.bernoulli.rvs(p=sensor_b)
        elif occurrence == 0: # white
            return 1 - stats.bernoulli.rvs(p=sensor_w)

    def local_estimate(self, obs: Observation, sensor_b: float, sensor_w: float):
        """Local estimation of the fill ratio.

        Returns:
            x_hat: Local estimate.
            alpha: Local confidence.
        """

        q_sqrt = sensor_b + sensor_w - 1.0 # common term

        if obs.n < 0 or obs.n > obs.t: raise RuntimeError("Observed values are erroneous")

        regular_estimate = (obs.n / obs.t + sensor_w - 1.0) / q_sqrt

        if regular_estimate <= 0.0:
            x_hat = 0.0
            alpha = q_sqrt**2 * (obs.t * sensor_w**2 - (2 * sensor_w - 1.0) * obs.complement) / (sensor_w**2 * (sensor_w - 1.0)**2)
        elif regular_estimate >= 1.0:
            x_hat = 1.0
            alpha = q_sqrt**2 * (obs.t * sensor_b**2 - 2 * obs.n * sensor_b + obs.n) / (sensor_b ** 2 * (sensor_b - 1.0)**2)
        else:
            x_hat = regular_estimate
            alpha = q_sqrt**2 * obs.t**3 / (obs.n * obs.complement)

        # Prevent numerical errors
        if alpha == 0: alpha = 1e-6

        return x_hat, alpha
    
    def social_estimate(self, x_hat_arr: np.array, alpha_arr: np.array):
        """Social estimation of the fill ratio.

        Returns:
            x_bar: Weighted average of neighbor estimates.
            beta: Sum of neighbor confidences.
        """
        x_bar = np.average(x_hat_arr, weights=alpha_arr)
        beta = sum(alpha_arr)

        return x_bar, beta

    def informed_estimate(self, x_hat, alpha, x_bar, beta):
        """Informed estimation of the fill ratio.
        """
        return (alpha * x_hat + beta * x_bar) / (alpha + beta)

class Robot(MinimalisticCollectivePerception):

    """
    This class is for the single robot that estimates its sensor degradation level
    """

    def __init__(self,
                 A,
                 B,
                 R,
                 act_sensor_quality: np.ndarray,
                 est_sensor_quality_init: np.ndarray,
                 est_sensor_quality_cov_init: np.ndarray,
                 use_obs_queue=False,
                 obs_queue_size=10):
        
        if type(A) == np.ndarray and A.size == 2:
            self.sensor_filter = SensorFilter2D(A, R) # @todo: implement usage of B matrix
        elif type(A) == float or type(A) == int:
            self.sensor_filter = SensorFilter1D(A, B, R)
        self.min_cp_solver = MinimalisticCollectivePerception()
        self.act_sensor_quality = act_sensor_quality # [[sensor_b], [sensor_w]]
        self.est_sensor_quality = Estimate(est_sensor_quality_init, est_sensor_quality_cov_init)

        self.x_hat = 0.5
        self.alpha = 0
        self.x_bar = 0.5
        self.beta = 0
        self.x = 0.5
        self.use_obs_queue = use_obs_queue
        self.obs = Observation(0, 0, use_queue=use_obs_queue, queue_size=obs_queue_size if self.use_obs_queue else 0)

    def evolve_sensor_degradation(self):
        self.act_sensor_quality = self.sensor_filter.degrade_quality(self.act_sensor_quality)

    def compute_local_estimate(self, occurrence: int):

        observation = super().make_observation(
                occurrence,
                self.act_sensor_quality[0,0], # observe with actual sensor qualitiy
                self.act_sensor_quality[1,0]
            )

        if self.use_obs_queue:
            self.obs.obs_queue.append(observation)
            self.obs.update_obs_from_queue()
            self.obs.complement = self.obs.t - self.obs.n
        else:
            self.obs.n += super().make_observation(
                occurrence,
                self.act_sensor_quality[0,0], # observe with actual sensor qualitiy
                self.act_sensor_quality[1,0]
            )
            self.obs.t += 1
            self.obs.complement = self.obs.t - self.obs.n

        self.x_hat, self.alpha = super().local_estimate(
            self.obs,
            self.est_sensor_quality.x[0,0], # estimate with estimated sensor quality
            self.est_sensor_quality.x[1,0]
        )

    def compute_social_estimate(self, x_hat_arr: np.array, alpha_arr: np.array):
        self.x_bar, self.beta = super().social_estimate(x_hat_arr, alpha_arr)

    def compute_informed_estimate(self):
        self.x = super().informed_estimate(self.x_hat, self.alpha, self.x_bar, self.beta)

    def get_observations(self):
        return self.obs.n, self.obs.t

    def get_local_estimate(self):
        return self.x_hat, self.alpha

    def get_social_estimate(self):
        return self.x_bar, self.beta

    def get_informed_estimate(self):
        return self.x

    def run_sensor_degradation_filter(self, elapsed_duration):
        self.est_sensor_quality, result = self.sensor_filter.estimate(
            self.est_sensor_quality,
            self.obs,
            self.x,
            elapsed_duration
        )
        return result
    
    def get_est_sensor_quality(self):
        return self.est_sensor_quality
    
    def get_act_sensor_quality(self):
        return self.act_sensor_quality
    
class RobotStaticDegradation(Robot):

    """
    This class is for the single robot that estimates its sensor degradation level while assuming it doesn't continue to degrade
    """

    def __init__(self,
                 filter_type: str,
                 act_sensor_quality: np.ndarray,
                 est_sensor_quality_init: np.ndarray,
                 est_sensor_quality_cov_init: np.ndarray,
                 filter_params=[]):
        
        super().__init__(1, 1, act_sensor_quality, est_sensor_quality_init, est_sensor_quality_cov_init)

        self.filter_type = filter_type
        if filter_type == "ALPHA":
            self.sensor_filter = SensorFilter1DAlpha()
        elif filter_type == "BRAVO":
            self.type_2_err_prob = filter_params[0]
            self.sensor_filter = SensorFilter1DAlpha()

    def evolve_sensor_degradation(self):
        pass # override: do nothing

    def run_sensor_degradation_filter(self):
        
        if self.filter_type == "ALPHA":
            self.est_sensor_quality, result = self.sensor_filter.estimate(
                self.est_sensor_quality,
                self.obs,
                self.x_bar,
            )
        elif self.filter_type == "BRAVO":
            run_filter = self.compute_decision_to_run_filter()

            if (run_filter):
                self.est_sensor_quality, result = self.sensor_filter.estimate(
                    self.est_sensor_quality,
                    self.obs,
                    self.x_bar,
                )
            else:
                result = -1

        return result

    def compute_social_estimate(self, x_hat_arr: np.array, alpha_arr: np.array):
        super().compute_social_estimate(x_hat_arr, alpha_arr)

        if self.filter_type == "BRAVO":
            # Compute sample mean and sample variance
            self.x_sample_mean = np.mean(x_hat_arr)
            self.x_sample_std = np.sqrt(np.var(x_hat_arr, ddof=1))
            self.num_neighbors = len(x_hat_arr)

    def get_est_sensor_quality(self):
        return self.est_sensor_quality

    def get_act_sensor_quality(self):
        return self.act_sensor_quality

    def compute_decision_to_run_filter(self):

        if self.num_neighbors <= 1: return False

        score = stats.t.isf((1-self.type_2_err_prob) / 2, df=self.num_neighbors)

        # Compute one-sided interval
        one_sided_interval = self.x_sample_std * (score / np.sqrt(self.num_neighbors) + 1.0)
        return False if abs(self.x_sample_mean - self.x_hat) < one_sided_interval else True