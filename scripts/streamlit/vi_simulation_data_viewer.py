import streamlit as st
import pickle
import numpy as np
import sys
import matplotlib.pyplot as plt
<<<<<<< HEAD
sys.path.append("../../")

from notebooks.notebook_modules.general_module import (Estimate,
=======

# For streamlit cloud
sys.path.append("/mount/src/sensor-degradation-filter/")

from notebooks.general_module import (Estimate,
>>>>>>> Restructured directories.
                            Observation,
                            prediction_step,
                            create_surface_plot,
                            create_contour_plot,
                            ZERO_APPROX)

# Function to convert numpy ndarray to LaTeX
def np2latex_2d(mat):
    latex_str = np.array2string(mat, formatter={"float_kind":lambda x: "{:.2f}".format(x)}, max_line_width=np.inf)
    latex_str = latex_str.replace("[[", "$\\begin{bmatrix}")
    latex_str = latex_str.replace("]]", "\\end{bmatrix}$")
    latex_str = latex_str.replace(" ", "&")
    latex_str = latex_str.replace("]", "\\\ ")
    latex_str = latex_str.replace("&[", "")
    return latex_str

# Modify page layout
css='''
<style>
    section.main > div {max-width:85rem}
</style>
'''
st.markdown(css, unsafe_allow_html=True)

st.header("Visualization of VI-based sensor degradation filter data")
st.write("You can compare the approximated distribution found using variational inference with \
          the target posterior based on simulated data with different experimental parameters.")

# Load data
pickled_files = [
    # "sim_data_VI_121723_234636.sdv", # set 1
    # "sim_data_VI_121723_235437.sdv", # set 1
    # "sim_data_VI_121823_012251.sdv", # set 2
    # "sim_data_VI_121823_085151.sdv", # set 2
    # "sim_data_VI_121823_092824.sdv", # set 2
    # "sim_data_VI_121823_142720.sdv", # set 2
    "scripts/streamlit/sim_data_VI_122023_002701.sdv" # standalone
]

import numpy as np

# Combine all the data into a single object
base_obj = False

for f in pickled_files:
    with open(f, "rb") as fopen:
        obj = pickle.load(fopen)

    if not base_obj: base_obj = obj
    else:
        # Expand base object with data from subsequent objects
        target_pc_obj = obj.get_param_combo()
        base_obj.get_param_combo().update_params(
            target_pc_obj.mu,
            target_pc_obj.sigma,
            target_pc_obj.curr_obs,
            target_pc_obj.past_f
        )
        
        _ = [base_obj.store_data_pair(param, t_dist, s_dist, result) for param, (t_dist, s_dist, result) in obj.get_data_pair().items()]

A = base_obj.A
R = base_obj.R
pc_obj = base_obj.get_param_combo()

# Set up model parameter section
st.subheader("Fixed model parameters")
st.write("Sensor degradation model $A =$ ${0}$, sensor noise model $R =$ ${1}$."
         .format(np2latex_2d(A), np2latex_2d(R)))

# Set up input section
st.write("##")
st.subheader("Experimental parameters")
col1_in, col2_in = st.columns(2)

# Past posterior mean
mu_latex_dict = {np2latex_2d(arr.reshape((2,1))): pc_obj.mu[ind] for ind, arr in enumerate(pc_obj.mu)}

with col1_in:
    mu_label = st.radio(
        "Select previous mean $\mu_{t-1}$",
        options=mu_latex_dict.keys(),
        horizontal=True
    )

# Past posterior covariance
sigma_latex_dict = {np2latex_2d(arr): pc_obj.sigma[ind] for ind, arr in enumerate(pc_obj.sigma)}

with col2_in:
    sigma_label = st.radio(
        "Select previous covariance $\Sigma_{t-1}$",
        options=sigma_latex_dict.keys(),
        horizontal=True
    )

prediction = prediction_step(
    Estimate(mu_latex_dict[mu_label], sigma_latex_dict[sigma_label]),
    A,
    R
)

# Predicted values
with col1_in:
    st.write("Predicted mean $A \mu_{{t-1}} =$ ${0}$".format(np2latex_2d(prediction.x.reshape(2,1))))

with col2_in:
    st.write("Predicted covariance $A \Sigma_{{t-1}} A^\\top + R =$ ${0}$".format(np2latex_2d(prediction.covar)))

# Observed values
obs_latex_dict = {
    "$(t={0}, n={1})$".format(obs.t, obs.n): pc_obj.curr_obs[ind] for ind, obs in enumerate(pc_obj.curr_obs)
}

with col1_in:
    obs_label = st.radio(
        "Select observation pair $(t,n)$",
        options=obs_latex_dict.keys(),
        horizontal=True
    )

f_latex_dict = {"$f={0}$".format(val): pc_obj.past_f[ind] for ind, val in enumerate(pc_obj.past_f)}

with col2_in:
    f_label = st.radio(
        "Select fill ratio $f$",
        options=f_latex_dict.keys(),
        horizontal=True
    )

data_pairs = base_obj.get_data_pair()

# Extract the distribution data
joint_dist = []
surr_dist = []

for est_tup, obs_tup, past_f in data_pairs.keys():

    est = Estimate(*est_tup)
    obs = Observation(*obs_tup)

    if past_f == f_latex_dict[f_label]:
        if (obs.n == obs_latex_dict[obs_label].n and
            obs.t == obs_latex_dict[obs_label].t):
            if (np.array_equal(est.x, mu_latex_dict[mu_label]) and
                np.array_equal(est.covar, sigma_latex_dict[sigma_label])):

                joint_dist, surr_dist, result = data_pairs[(est_tup, obs_tup, past_f)]

                break

# Create container for chart output
st.write("##")

output_container = st.container()

with output_container:
    st.subheader("Comparison of the approximated distribution with the posterior")
    col1_out, col2_out = st.columns(2)
    support_step_size = 1e-3
    support = np.arange(ZERO_APPROX, 1-ZERO_APPROX, support_step_size) # n points
    x, y = np.meshgrid(support, support)
    pos = np.vstack((x.flatten(), y.flatten())).T # (n*n) x 2 vector of coordinate points
    xlab = r"$\theta_1$"
    ylab = r"$\theta_2$"

    # Joint distribution (posterior); normalized to support
    fig1, ax1 = plt.subplots(1, 2, figsize=(10, 6), subplot_kw={"projection": "3d"})

    ax1[0] = create_surface_plot(
        ax1[0],
        x,
        y,
        joint_dist.pdf(pos).reshape(x.shape),
        xlab,
        ylab,
        "Normalized Posterior"
    )

    ax1[1] = create_surface_plot(
        ax1[1],
        x,
        y,
        surr_dist.pdf(pos).reshape(x.shape),
        xlab,
        ylab,
        "Surrogate distribution\n(Bivariate Normal)"
    )

    with col1_out:
        st.pyplot(fig1)
        # st.write("$\ln p(z) = {0}$".format(np.log(joint_dist.norm_const)))
        # st.write("KL divergence $= {0}$"
        #          .format(np.log(joint_dist.norm_const) - (-result[-1].fun)))

    # Joint distribution (posterior); normalized to support
    fig2, ax2 = plt.subplots(1, 2, figsize=(10, 6))

    ax2[0] = create_contour_plot(
        ax2[0],
        x,
        y,
        joint_dist.pdf(pos).reshape(x.shape),
        xlab,
        ylab,
        "Normalized Posterior"
    )

    # Surrogate distribution; normalized to support
    ax2[1] = create_contour_plot(
        ax2[1],
        x,
        y,
        surr_dist.pdf(pos).reshape(x.shape),
        xlab,
        ylab,
        "Surrogate Distribution\n(Bivariate Normal)"
    )

    with col2_out:
        st.pyplot(fig2)