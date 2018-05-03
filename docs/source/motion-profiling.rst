Motion Profiling
================

What is it?
+++++++++++++++++

Motion profiling is a Feed-Forward system where you calculate the state of the robot (distance, direction, power) at very small intervals of time (5-20ms).

Part 1
++++++

Here is some code::

	public class Factorial
	{
		public static void main(String[] args)
		{	final int NUM_FACTS = 100;
			for(int i = 0; i < NUM_FACTS; i++)
				System.out.println( i + "! is " + factorial(i));
		}

		public static int factorial(int n)
		{	int result = 1;
			for(int i = 2; i <= n; i++)
				result *= i;
			return result;
		}
	}

Part 2
++++++

.. note:: Important stuff, learn harder.

+------------+--------------+-------------------------------------+
| Subsystem  | Preferred    | Reason                              |
+------------+--------------+-------------------------------------+
| Drivetrain | PathFinder   | 2d path, benefits from more sensors |
+------------+--------------+-------------------------------------+
| Lift       | Motion Magic | 1d path, off load to Talon SRX      |
+------------+--------------+-------------------------------------+
| Intake     | Bang Bang    | No control needed, on/off           |
+------------+--------------+-------------------------------------+




Resources
+++++++++
:download:`Path Generator</files/Motion Profile Generator New.jar>`

:download:`Drive Train Characterization whitepaper</files/motor_characterization.pdf>`

