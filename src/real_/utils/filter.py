import numpy as np
from scipy import spatial
from pykalman import KalmanFilter as KFilter

# TODO: is Kalman filter really the way to go for us?


class Filter(object):
    """
    A base class for filtering a noisy data stream in an online fashion.
    """
    def __init__(self):
        pass

    def estimate(self, observation):
        """
        Takes an observation and returns a de-noised estimate.

        :param observation: A current observation.
        :return: De-noised estimate.
        """
        raise NotImplementedError

class KalmanFilter(Filter):

    # TODO: there's a lot to tweak here, like including / estimating offsets, whether EM can even help, etc..

    """
    This class uses a Kalman Filter to de-noise a noisy data stream in an online fashion.
    """
    def __init__(self, obs_dim, measurements=None):
        """
        Initialize a Kalman Filter. If measurements are provided, also estimate
        the model parameters.

        :param obs_dim: The dimension of the points to filter.
        :param measurements: If provided, estimate the model parameters first
                             using EM. This should be a list of state measurements.
        """
        self.obs_dim = obs_dim
        self.kf = KFilter(n_dim_state=self.obs_dim, n_dim_obs=self.obs_dim)

        if measurements is not None:
            self.kf.em(measurements)

        # keep online estimates of hidden state means and covariances
        self.filtered_state_mean = np.zeros(self.obs_dim)
        self.filtered_state_covariance = np.eye(self.obs_dim)

        super(KalmanFilter, self).__init__()

    def estimate(self, observation):
        """
        Do an online hold for state estimation given a recent observation.

        :param observation: New observation to hold internal estimate of state.
        :return: New estimate of state.
        """
        self.filtered_state_mean, self.filtered_state_covariance = self.kf.filter_update(self.filtered_state_mean,
                                                                                         self.filtered_state_covariance,
                                                                                         observation)
        return self.filtered_state_mean

class MovingAverageFilter(Filter):
    """
    This class uses a moving average to de-noise a noisy data stream in an online fashion.
    """
    def __init__(self, obs_dim, filter_width):
        """
        :param obs_dim: The dimension of the points to filter.
        :param filter_width: The number of past samples to take the moving average over.
        """
        self.filter_width = filter_width
        self.past_samples = []
        self.past_samples_sum = np.zeros(obs_dim)
        self.num_samples = 0

        super(MovingAverageFilter, self).__init__()

    def estimate(self, observation):
        """
        Do an online hold for state estimation given a recent observation.

        :param observation: New observation to hold internal estimate of state.
        :return: New estimate of state.
        """
        if self.num_samples == self.filter_width:
            val = self.past_samples.pop(0)
            self.past_samples_sum -= val
            self.num_samples -= 1
        self.past_samples.append(observation)
        self.past_samples_sum += observation
        self.num_samples += 1

        return self.past_samples_sum / self.num_samples



class Subsampler(object):
    """
    A base class for subsampling a data stream in an online fashion.
    """
    def __init__(self):
        pass

    def subsample(self, observation):
        """
        Takes an observation and returns the observation, or None, which
        corresponds to deleting the observation.

        :param observation: A current observation.
        :return: The observation, or None.
        """
        raise NotImplementedError

class UniformSubsampler(Subsampler):
    """
    A class for subsampling a data stream uniformly in time in an online fashion.
    """
    def __init__(self, T):
        """
        :param T: Pick one every T observations.
        """
        self.T = T
        self.counter = 0

        super(UniformSubsampler, self).__init__()

    def subsample(self, observation):
        """
        Returns an observation once every T observations, None otherwise.

        :param observation: A current observation.
        :return: The observation, or None.
        """
        self.counter += 1
        if self.counter == self.T:
            self.counter = 0
            return observation
        return None

class TrajectorySubsampler(Subsampler):
    """
    A class for subsampling a data stream that represents a trajectory in an
    online fashion. It subsamples the data by using the last 3 observations seen.
    Denote these observations (o1, o2, o3). If the cosine distance between (o2 - o1)
    and (o3 - o2) is large enough, that means that o3 makes a significant
    contribution to the shape of the trajectory, and we keep the point. We make this
    decision with respect to a threshold.
    """
    def __init__(self, threshold):
        """
        :param threshold: A small value (between 0.0, 2.0). Corresponds to
                          the minimum cosine distance value at which
                          to retain the most recent trajectory sample.
        """
        self.threshold = threshold
        self.past_samples = []

        super(TrajectorySubsampler, self).__init__()

    def subsample(self, observation):
        """
        Filters a point along a trajectory based on the previous two points
        in the trajectory. We take in points (x1, x2, x3) and decide whether
        to keep point x3 based on the cosine distance between (x3 - x2) and
        (x2 - x1), and the passed threshold value.

        :param observation: A current observation.
        :return: The observation, or None.
        """
        if len(self.past_samples) < 2:
            self.past_samples.append(observation)
            return observation

        diff_1 = self.past_samples[1] - self.past_samples[0]
        diff_2 = observation - self.past_samples[1]

        # Cosine distance has problems with zero vectors. Here,
        # if the first difference is zero, then keep the point only
        # if the second difference is non-zero.
        if not np.count_nonzero(diff_1):
            return np.count_nonzero(diff_2) > 0

        # Compute cosine distance.
        cos_dist = spatial.distance.cosine(diff_1, diff_2)

        # Update cache of past samples.
        self.past_samples.pop(0)
        self.past_samples.append(observation)

        if cos_dist <= self.threshold:
            return None
        return observation


class Interpolator(object):
    """
    This class implements methods for interpolation.
    """
    def __init__(self):
        pass

    def dynamic_linear_interpolation(self, initial, final, max_steps):
        """
        Performs a linear interpolation in each dimension between an initial
        vector and a final vector with a dynamically set step size. The
        step size is determined by the passed max_steps vector, which gives
        the maximum allowed step in each dimension. If there is no bound in a
        particular dimension, set that component to -1.

        :param initial: Initial position, a 1-D numpy array.
        :param final: Final position, a 1-D numpy array.
        :param max_steps: Max step sizes in each dimension, a 1-D numpy array. If there
                          is no constraint in a dimension, pass None for that entry.
                          All entries must be positive or None.
        :return: A tuple consisting of a 2-D numpy array representing the interpolation
                 and the number of steps taken between the initial and final positions.
        """
        assert isinstance(initial, np.ndarray), "initial vector must be 1-D numpy array"
        assert isinstance(final, np.ndarray), "final vector must be 1-D numpy array"
        assert isinstance(max_steps, np.ndarray), "array of max_steps must be 1-D numpy array"

        inds = max_steps < 0
        max_steps[inds] = final[inds] - initial[inds] # corresponds to single step, no constraint

        # since we are doing uniform steps, take a maximum over the minimum required steps
        # in each dimension to satisfy all of the constraints
        num_steps = int(np.max(np.ceil(np.abs(final - initial) / max_steps)))

        return self.linear_interpolation(initial, final, num_steps)

    def linear_interpolation(self, initial, final, num_steps):
        """
        Performs a linear interpolation in each dimension between an initial
        vector and a final vector according to the desired number of steps.

        :param initial: Initial position, a 1-D numpy array.
        :param final: Final position, a 1-D numpy array.
        :param num_steps: Number of steps for linear interpolation.
        :return: A 2-D numpy array of shape [num_steps+1, state_dim] where
                 where state_dim is the dimension of the initial and final
                 positions.
        """
        assert isinstance(initial, np.ndarray), "initial vector must be 1-D numpy array"
        assert isinstance(final, np.ndarray), "final vector must be 1-D numpy array"

        interp = np.zeros((num_steps + 1, len(initial)))
        for i in range(len(initial)):
            interp[:, i] = np.linspace(initial[i], final[i], num_steps + 1)
        return interp


if __name__ == "__main__":
    # f = KalmanFilter(3)
    # print(f.estimate([1, 1, 1]))
    # print(f.estimate([1, 1, 1]))
    # print(f.estimate([1, 1, 1]))
    # print(f.estimate([1, 1, 1]))
    # print(f.estimate([1, 1, 1]))
    # print(f.estimate([1, 1, 1]))
    # print(f.estimate([1, 1, 1]))

    # TODO: put a pause on the robot
    # TODO: try interpolation (in joint space) for smooth motion, try planning / look into planning algo
    # TODO: try subsampling, control incoming rate somehow...
    # TODO: try vanilla Kalman Filter, with EM, simple LPF
    # TODO: see hopper results, investigate normalized results...

    f = MovingAverageFilter(3, 10)
    # f = KalmanFilter(3)
    a = np.array([1, 1, 1])
    for i in range(500):
        print(f.estimate(a + np.random.normal(scale=0.1)))

    # i = Interpolator()
    # mat = i.dynamic_linear_interpolation(initial=np.arange(5), final=np.arange(5, 0, -1),
    #                                      max_steps=np.array([-1, -1, -1, -1, 0.1]))
    # print(mat)
    # print(mat.shape)


