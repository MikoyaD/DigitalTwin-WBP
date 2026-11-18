"""Focused, ROS-free checks for the six-panel plot's dual axes."""
import unittest

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from v2.scripts.plot_trajectory import align_base_ticks, plot_pair


class PlotDualAxisTests(unittest.TestCase):
    def test_legend_owns_upper_axes_and_matches_arm_translucency(self):
        # Theta enters the top-right corner where the original left-Axes
        # legend could be overprinted by the subsequently drawn twin Axes.
        t = np.linspace(0, 1, 101)
        data = {'time_s': t, 'q_world_car/x': t * 0,
                'q_world_car/y': t * 0.1,
                'q_world_car/theta': t * 5}
        for i in range(1, 7):
            data[f'q_joint{i}'] = t * 0
        fig, axes = plt.subplots(1, 2)
        right = plot_pair(axes[0], axes[1], data, 'q')
        fig.canvas.draw()
        legend = right.get_legend()
        self.assertIsNotNone(legend)
        self.assertIsNone(axes[0].get_legend())
        self.assertIs(fig.axes[-1], right)
        self.assertFalse(right.patch.get_visible())
        self.assertEqual(legend.get_frame().get_alpha(),
                         axes[1].get_legend().get_frame().get_alpha())
        self.assertLess(legend.get_frame().get_alpha(), 1.0)
        self.assertEqual(legend.get_frame().get_facecolor()[:3], (1.0, 1.0, 1.0))
        self.assertEqual(len(legend.get_lines()), 3)
        plt.close(fig)

    def test_major_ticks_match_display_heights_without_changing_limits(self):
        fig, left = plt.subplots()
        right = left.twinx()
        for llim, rlim in [((-1.7, 3.4), (-.02, 4.7)),
                           ((-0.0003, .0017), (-5.1, .9)),
                           ((12.1, 25.3), (-2.3, -0.1))]:
            left.set_ylim(*llim)
            right.set_ylim(*rlim)
            fig.canvas.draw()
            before = (left.get_ylim(), right.get_ylim())
            lt, rt = align_base_ticks(left, right)
            fig.canvas.draw()
            self.assertEqual(before, (left.get_ylim(), right.get_ylim()))
            self.assertEqual(len(lt), len(right.get_yticks()))
            self.assertTrue(np.allclose(rt, right.get_yticks()))
            ly = left.transData.transform(np.column_stack((np.zeros(len(lt)), lt)))[:, 1]
            ry = right.transData.transform(np.column_stack((np.zeros(len(rt)), rt)))[:, 1]
            np.testing.assert_allclose(ly, ry, atol=1e-8)
            self.assertTrue(np.all(np.diff(rt) > 0))
        plt.close(fig)


if __name__ == '__main__':
    unittest.main()
