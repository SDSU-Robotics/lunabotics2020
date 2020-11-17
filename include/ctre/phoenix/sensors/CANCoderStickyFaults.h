#pragma once

namespace ctre {
	namespace phoenix {
		namespace sensors {

			/**
			 * Sticky Faults for CANCoder (Currently has none)
			 */
			struct CANCoderStickyFaults {
				/**
				 * @return true if any faults are tripped
				 */
				bool HasAnyFault() const {
					return false;
				}
				/**
				 * @return Current fault list as a bit field
				 */
				int ToBitfield() const {
					int retval = 0;
					return retval;
				}
				/**
				 * Updates current fault list with specified bit field of faults
				 *
				 * @param bits bit field of faults to update with
				 */
				CANCoderStickyFaults(int bits) {
					(void)bits;
				}
				CANCoderStickyFaults() {
				}
			};

		} // namespace sensors
	} // namespace phoenix
} // namespace ctre
