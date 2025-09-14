/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef PIREGULATOR_H
#define PIREGULATOR_H

#include "my_fp.h"
#include "my_math.h"

template<typename Tin, typename Tout>
class PiControllerGeneric
{
   public:
      /** Default constructor */
      PiControllerGeneric()
      : kp(0), ki(0), esum(0), refVal(0), frequency(1), maxY(0), minY(0) {}

      /** Set regulator proportional and integral gain.
       * \param kp New value to set for proportional gain
       * \param ki New value for integral gain
       */
      void SetGains(Tout kp, Tout ki)
      {
         SetProportionalGain(kp);
         SetIntegralGain(ki);
      }

      void SetProportionalGain(Tout kp) { this->kp = kp; }
      void SetIntegralGain(Tout ki);

      /** Set regulator target set point
       * \param val regulator target
       */
      void SetRef(Tin val) { refVal = val; }

      Tin GetRef() { return refVal; }

      /** Set maximum controller output
        * \param val actuator saturation value
        */
      void SetMinMaxY(Tout valMin, Tout valMax)
      { minY = valMin; maxY = valMax; SetIntegralGain(ki); }

      /** Set calling frequency
       * \param val New value to set
       */
      void SetCallingFrequency(int val) { frequency = val; SetIntegralGain(ki); }

      /** Run controller to obtain a new actuator value
       * \param curVal currently measured value
       * \return new actuator value
       */
      Tout Run(Tin curVal, Tout feedForward = 0);

      /** Run controller to obtain a new actuator value, run only proportional part
       * \param curVal currently measured value
       * \return new actuator value
       */
      Tout RunProportionalOnly(Tin curVal);

      /** Reset integrator to 0 */
      void ResetIntegrator() { esum = 0; }

      /** Preload Integrator to yield a certain output
       * @pre SetCallingFrequency() and SetGains() must be called first
      */
      void PreloadIntegrator(Tout yieldedOutput) { esum = ki != 0 ? FP_FROMINT((yieldedOutput * frequency) / ki) : 0; }

      /** Debug function for getting integrator */
      Tin GetIntegrator() { return esum; }

   protected:

   private:
      Tout kp; //!< Proportional controller gain
      Tout ki; //!< Integral controller gain
      Tin esum; //!< Integrator
      Tin refVal; //!< control target
      Tout frequency; //!< Calling frequency
      Tout maxY; //!< upper actuator saturation value
      Tout minY; //!< lower actuator saturation value
      Tout minSum; //!< upper integrator boundary
      Tout maxSum; //!< lower integrator boundary
};

typedef PiControllerGeneric<s32fp, int32_t> PiController;
typedef PiControllerGeneric<float, float> PiControllerFloat;
#endif // PIREGULATOR_H
