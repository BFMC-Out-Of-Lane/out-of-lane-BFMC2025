import numpy as np


class Line:
    def __init__(self, n_frames=5, max_missing=5):
        self.n_frames = n_frames       # Number of frames to keep
        self.max_missing = max_missing  # Max missing frames before reset
        self.detected = False          # Was the lane detected in the last iteration?
        self.recent_fits = []          # Polynomial coefficients of the last n fits
        self.best_fit = None           # Averaged polynomial coefficients
        self.missing_count = 0         # Count of consecutive missing detections

    def add_fit(self, fit):
        if fit is not None:
            self.detected = True
            self.missing_count = 0
            self.recent_fits.append(fit)
            if len(self.recent_fits) > self.n_frames:
                # Keep only the most recent n frames
                self.recent_fits = self.recent_fits[-self.n_frames:]
            # Compute the average fit
            self.best_fit = np.mean(self.recent_fits, axis=0)
        else:
            self.detected = False
            self.missing_count += 1
            if self.missing_count >= self.max_missing:
                # Reset the lane data after too many misses
                self.recent_fits = []
                self.best_fit = None
                self.missing_count = 0
