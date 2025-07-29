import numpy as np
from scipy import signal
from scipy.optimize import curve_fit
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from typing import Tuple, Optional, Dict, List


class TwoParabolaFitter:
    """
    Fits two parabolas to TOF data vs joint angles, handling angular wraparound.
    Maintains index tracking for timestamp correlation.
    """

    def __init__(
        self,
        far_plane_threshold: float = 0.25,
        angular_tolerance: float = np.pi / 6,  # 30 degrees
        min_points_per_parabola: int = 10,
    ):
        self.far_plane_threshold = far_plane_threshold
        self.angular_tolerance = angular_tolerance
        self.min_points_per_parabola = min_points_per_parabola

    def normalize_angles(self, angles: np.ndarray) -> np.ndarray:
        """Normalize angles to [-π, π] range"""
        return np.arctan2(np.sin(angles), np.cos(angles))

    def angular_distance(self, a1: float, a2: float) -> float:
        """Calculate shortest angular distance between two angles"""
        diff = abs(a1 - a2)
        return min(diff, 2 * np.pi - diff)

    def find_minima_with_wraparound(
        self, angles: np.ndarray, tof_data: np.ndarray, original_indices: np.ndarray
    ) -> List[Dict]:
        """
        Find minima in TOF data, handling angular wraparound properly.
        Returns list of minima with their properties.
        """
        # Filter out far plane data first
        valid_mask = tof_data <= self.far_plane_threshold
        if not np.any(valid_mask):
            return []

        valid_angles = angles[valid_mask]
        valid_tof = tof_data[valid_mask]
        valid_indices = original_indices[valid_mask]

        # Find peaks in inverted data (minima become maxima)
        peak_indices, properties = signal.find_peaks(
            -valid_tof,
            distance=max(5, len(valid_tof) // 20),  # Adaptive distance
            prominence=0.01,  # Minimum prominence to avoid noise
        )

        minima = []
        for peak_idx in peak_indices:
            minima.append(
                {
                    "angle": valid_angles[peak_idx],
                    "tof": valid_tof[peak_idx],
                    "local_index": peak_idx,  # Index in filtered data
                    "original_index": valid_indices[peak_idx],  # Original data index
                    "angle_normalized": self.normalize_angles(valid_angles[peak_idx]),
                }
            )

        # Check endpoints as potential minima
        if len(valid_tof) > 2:
            # Check first point
            if valid_tof[0] < valid_tof[1]:
                minima.append(
                    {
                        "angle": valid_angles[0],
                        "tof": valid_tof[0],
                        "local_index": 0,
                        "original_index": valid_indices[0],
                        "angle_normalized": self.normalize_angles(valid_angles[0]),
                    }
                )

            # Check last point
            if valid_tof[-1] < valid_tof[-2]:
                minima.append(
                    {
                        "angle": valid_angles[-1],
                        "tof": valid_tof[-1],
                        "local_index": len(valid_tof) - 1,
                        "original_index": valid_indices[-1],
                        "angle_normalized": self.normalize_angles(valid_angles[-1]),
                    }
                )

        return minima

    def cluster_minima_by_angular_proximity(self, minima: List[Dict]) -> List[Dict]:
        """
        Cluster minima by angular proximity, handling wraparound.
        Returns representative minima (one per cluster).
        """
        if len(minima) <= 2:
            return minima

        # Create feature matrix for clustering
        # Use both sin and cos to handle wraparound properly
        angles = np.array([m["angle_normalized"] for m in minima])
        features = np.column_stack(
            [np.cos(angles), np.sin(angles), [m["tof"] for m in minima]]  # Include TOF for tie-breaking
        )

        # Use DBSCAN with custom metric for angular clustering
        clustering = DBSCAN(eps=0.3, min_samples=1, metric="euclidean").fit(  # Adjust based on angular_tolerance
            features
        )

        # Find best representative from each cluster
        clustered_minima = []
        for cluster_id in np.unique(clustering.labels_):
            if cluster_id == -1:  # Noise points
                continue

            cluster_mask = clustering.labels_ == cluster_id
            cluster_minima = [minima[i] for i in np.where(cluster_mask)[0]]

            # Choose the minimum with lowest TOF value in this cluster
            best_minimum = min(cluster_minima, key=lambda x: x["tof"])
            clustered_minima.append(best_minimum)

        return clustered_minima

    def unwrap_angles_for_section(self, angles: np.ndarray, indices: np.ndarray) -> np.ndarray:
        """
        Unwrap angles for a continuous section, handling gaps in indices.
        """
        if len(angles) <= 1:
            return angles.copy()

        # Find gaps in indices (indicates potential wraparound)
        index_diffs = np.diff(indices)
        large_gaps = np.where(index_diffs > np.median(index_diffs) * 3)[0]

        if len(large_gaps) == 0:
            # No large gaps, simple unwrap
            return np.unwrap(angles)

        # Handle sections separately across gaps
        unwrapped = angles.copy()
        start_idx = 0

        for gap_idx in large_gaps:
            end_idx = gap_idx + 1
            section = angles[start_idx:end_idx]
            if len(section) > 1:
                unwrapped[start_idx:end_idx] = np.unwrap(section)
            start_idx = end_idx

        # Handle final section
        if start_idx < len(angles):
            section = angles[start_idx:]
            if len(section) > 1:
                unwrapped[start_idx:] = np.unwrap(section)

        return unwrapped

    def segment_data_by_minima(
        self, angles: np.ndarray, tof_data: np.ndarray, original_indices: np.ndarray, minima: List[Dict]
    ) -> Dict[str, Dict]:
        """
        Segment data into two parts based on minima locations.
        """
        if len(minima) < 2:
            raise ValueError(f"Need at least 2 minima, found {len(minima)}")

        if len(minima) == 2:
            return self._segment_two_minima(angles, tof_data, original_indices, minima)
        else:
            # More than 2 minima - try to identify which ones to use
            return self._segment_multiple_minima(angles, tof_data, original_indices, minima)

    def _segment_two_minima(
        self, angles: np.ndarray, tof_data: np.ndarray, original_indices: np.ndarray, minima: List[Dict]
    ) -> Dict[str, Dict]:
        """Handle the simple case of exactly 2 minima"""

        # Sort minima by their original index (temporal order)
        minima_sorted = sorted(minima, key=lambda x: x["original_index"])

        # Find midpoint between minima in the original data
        idx1 = minima_sorted[0]["original_index"]
        idx2 = minima_sorted[1]["original_index"]

        # Find indices in our current arrays
        pos1 = np.where(original_indices == idx1)[0][0]
        pos2 = np.where(original_indices == idx2)[0][0]

        if pos1 > pos2:
            pos1, pos2 = pos2, pos1

        midpoint_pos = (pos1 + pos2) // 2

        # Create segments
        seg1_mask = np.arange(len(angles)) <= midpoint_pos
        seg2_mask = np.arange(len(angles)) >= midpoint_pos

        return {
            "segment_1": {
                "angles": angles[seg1_mask],
                "tof_data": tof_data[seg1_mask],
                "original_indices": original_indices[seg1_mask],
                "mask": seg1_mask,
            },
            "segment_2": {
                "angles": angles[seg2_mask],
                "tof_data": tof_data[seg2_mask],
                "original_indices": original_indices[seg2_mask],
                "mask": seg2_mask,
            },
        }

    def _segment_multiple_minima(
        self, angles: np.ndarray, tof_data: np.ndarray, original_indices: np.ndarray, minima: List[Dict]
    ) -> Dict[str, Dict]:
        """Handle case with more than 2 minima (likely due to wraparound)"""

        # Find the two minima that are farthest apart angularly
        max_distance = 0
        best_pair = None

        for i, m1 in enumerate(minima):
            for j, m2 in enumerate(minima[i + 1 :], i + 1):
                dist = self.angular_distance(m1["angle_normalized"], m2["angle_normalized"])
                if dist > max_distance:
                    max_distance = dist
                    best_pair = (m1, m2)

        if best_pair is None:
            raise ValueError("Could not identify primary minima pair")

        return self._segment_two_minima(angles, tof_data, original_indices, best_pair)

    def fit_parabola(self, angles: np.ndarray, tof_data: np.ndarray) -> Optional[Dict]:
        """
        Fit a parabola to the given data.
        Returns fit parameters and statistics.
        """
        if len(angles) < self.min_points_per_parabola:
            return None

        def parabola(x, a, b, c):
            return a * x**2 + b * x + c

        try:
            # Initial guess
            p0 = [1.0, 0.0, np.mean(tof_data)]

            popt, pcov = curve_fit(parabola, angles, tof_data, p0=p0)

            # Calculate fit statistics
            y_pred = parabola(angles, *popt)
            residuals = tof_data - y_pred
            ss_res = np.sum(residuals**2)
            ss_tot = np.sum((tof_data - np.mean(tof_data)) ** 2)
            r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0

            # Find vertex (minimum of parabola)
            vertex_angle = -popt[1] / (2 * popt[0]) if popt[0] != 0 else 0
            vertex_tof = parabola(vertex_angle, *popt)

            return {
                "coefficients": popt,
                "covariance": pcov,
                "r_squared": r_squared,
                "rmse": np.sqrt(np.mean(residuals**2)),
                "vertex_angle": vertex_angle,
                "vertex_tof": vertex_tof,
                "residuals": residuals,
                "predicted": y_pred,
            }

        except Exception as e:
            print(f"Parabola fitting failed: {e}")
            return None

    def fit_two_parabolas(
        self, angles: np.ndarray, tof_data: np.ndarray, original_indices: Optional[np.ndarray] = None
    ) -> Dict:
        """
        Main function to fit two parabolas to the data.

        Args:
            angles: Joint angles (radians)
            tof_data: Time-of-flight measurements
            original_indices: Original indices for timestamp correlation

        Returns:
            Dictionary containing fit results and segmented data
        """
        if original_indices is None:
            original_indices = np.arange(len(angles))

        # Step 1: Find minima
        minima = self.find_minima_with_wraparound(angles, tof_data, original_indices)

        if len(minima) < 2:
            return {"success": False, "error": f"Insufficient minima found: {len(minima)}", "minima": minima}

        # Step 2: Cluster minima to handle duplicates
        clustered_minima = self.cluster_minima_by_angular_proximity(minima)

        if len(clustered_minima) < 2:
            return {
                "success": False,
                "error": f"Insufficient distinct minima: {len(clustered_minima)}",
                "minima": clustered_minima,
            }

        # Step 3: Segment data
        try:
            segments = self.segment_data_by_minima(angles, tof_data, original_indices, clustered_minima)
        except Exception as e:
            return {"success": False, "error": f"Segmentation failed: {str(e)}", "minima": clustered_minima}

        # Step 4: Unwrap angles for each segment
        for seg_name, seg_data in segments.items():
            seg_data["angles_unwrapped"] = self.unwrap_angles_for_section(
                seg_data["angles"], seg_data["original_indices"]
            )

        # Step 5: Fit parabolas
        results = {"success": True, "minima": clustered_minima, "segments": segments, "fits": {}}

        for seg_name, seg_data in segments.items():
            fit_result = self.fit_parabola(seg_data["angles_unwrapped"], seg_data["tof_data"])
            results["fits"][seg_name] = fit_result

            if fit_result is None:
                results["success"] = False
                results["error"] = f"Parabola fitting failed for {seg_name}"

        return results

    def plot_results(self, angles: np.ndarray, tof_data: np.ndarray, results: Dict, title: str = "Two Parabola Fit"):
        """Plot the original data, minima, segments, and fitted parabolas"""

        if not results["success"]:
            print(f"Cannot plot - fitting failed: {results.get('error', 'Unknown error')}")
            return

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

        # Plot 1: Original data with minima and segments
        ax1.scatter(angles, tof_data, alpha=0.6, s=20, c="lightgray", label="All data")

        # Plot minima
        for i, minimum in enumerate(results["minima"]):
            ax1.scatter(minimum["angle"], minimum["tof"], s=100, c="red", marker="x", label="Minima" if i == 0 else "")

        # Plot segments
        colors = ["blue", "orange"]
        for i, (seg_name, seg_data) in enumerate(results["segments"].items()):
            ax1.scatter(seg_data["angles"], seg_data["tof_data"], c=colors[i], alpha=0.7, s=30, label=f"{seg_name}")

        ax1.set_xlabel("Joint Angle (rad)")
        ax1.set_ylabel("TOF Distance")
        ax1.set_title(f"{title} - Data Segmentation")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot 2: Fitted parabolas
        for i, (seg_name, seg_data) in enumerate(results["segments"].items()):
            fit_result = results["fits"][seg_name]
            if fit_result is not None:
                # Plot original data points
                ax2.scatter(
                    seg_data["angles_unwrapped"],
                    seg_data["tof_data"],
                    c=colors[i],
                    alpha=0.7,
                    s=30,
                    label=f"{seg_name} data",
                )

                # Plot fitted parabola
                x_fit = np.linspace(seg_data["angles_unwrapped"].min(), seg_data["angles_unwrapped"].max(), 100)

                def parabola(x, a, b, c):
                    return a * x**2 + b * x + c

                y_fit = parabola(x_fit, *fit_result["coefficients"])
                ax2.plot(
                    x_fit, y_fit, c=colors[i], linewidth=2, label=f'{seg_name} fit (R²={fit_result["r_squared"]:.3f})'
                )

                # Mark vertex
                ax2.scatter(
                    fit_result["vertex_angle"],
                    fit_result["vertex_tof"],
                    c=colors[i],
                    marker="*",
                    s=150,
                    edgecolor="black",
                    label=f"{seg_name} vertex",
                )

        ax2.set_xlabel("Joint Angle (rad, unwrapped)")
        ax2.set_ylabel("TOF Distance")
        ax2.set_title(f"{title} - Fitted Parabolas")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()
