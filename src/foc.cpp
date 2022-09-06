/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
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
#define CST_DIGITS 15
#include "my_fp.h"
#include "my_math.h"
#include "foc.h"
#include "sine_core.h"

#define SQRT3 FP_FROMFLT(1.732050807568877293527446315059)

static const u32fp sqrt3 = SQRT3;
static const s32fp sqrt3inv1 = FP_FROMFLT(0.57735026919); //1/sqrt(3)
static const s32fp zeroOffset = FP_FROMINT(1);
static const int32_t modMax = FP_DIV(FP_FROMINT(2U), sqrt3);
static const int32_t modMaxPow2 = modMax * modMax;
static const int32_t minPulse = 1000;
static const int32_t maxPulse = FP_FROMINT(2) - 1000;

static float term1 = 15, term2 = 240;

s32fp FOC::id;
s32fp FOC::iq;
s32fp FOC::DutyCycles[3];
s32fp FOC::sin;
s32fp FOC::cos;

/** @brief Set angle for Park und inverse Park transformation
 *  @param angle uint16_t rotor angle
 */
void FOC::SetAngle(uint16_t angle)
{
   sin = SineCore::Sine(angle);
   cos = SineCore::Cosine(angle);
}

/** @brief Transform current to rotor system using Clarke and Park transformation
  * @pre Call SetAngle to specify angle for Park transformation
  * @post flux producing (id) and torque producing (iq) current are written
  *       to FOC::id and FOC::iq
  */
void FOC::ParkClarke(s32fp il1, s32fp il2)
{
   //Clarke transformation
   s32fp ia = il1;
   s32fp ib = FP_MUL(sqrt3inv1, il1 + 2 * il2);
   //Park transformation
   id = FP_MUL(cos, ia) + FP_MUL(sin, ib);
   iq = FP_MUL(cos, ib) - FP_MUL(sin, ia);
}

/** \brief distribute motor current in magnetic torque and reluctance torque with the least total current
 *
 * \param[in] is int32_t total motor current
 * \param[out] idref int32_t& resulting direct current reference
 * \param[out] iqref int32_t& resulting quadrature current reference
 *
 */
void FOC::Mtpa(float is, float& idref, float& iqref)
{
   float isSquared = is * is;

   idref = term1 == 0 ? 0 : 0.25f * (term1 - floatSqrt(term2 + 8 * isSquared));
   iqref = SIGN(is) * floatSqrt(isSquared - idref * idref);
}

void FOC::SetMotorParameters(float ldminuslq, float fluxLinkage)
{
   if (ldminuslq > 0)
   {
      term1 = fluxLinkage / ldminuslq;
      term2 = (fluxLinkage * fluxLinkage) / (ldminuslq * ldminuslq);
   }
   else
   {
      term1 = term2 = 0;
   }
}

int32_t FOC::GetQLimit(int32_t ud)
{
   return sqrt(modMaxPow2 - ud * ud);
}

/** \brief Returns the resulting modulation index from uq and ud
 *
 * \param ud d voltage modulation index
 * \param uq q voltage modulation index
 * \return sqrt(ud²+uq²)
 *
 */
int32_t FOC::GetTotalVoltage(int32_t ud, int32_t uq)
{
   return sqrt((uint32_t)(ud * ud) + (uint32_t)(uq * uq));
}

/** \brief Calculate duty cycles for generating ud and uq at given angle
 *
 * @pre Call SetAngle to specify angle for inverse Park transformation
 *
 * \param ud int32_t direct voltage
 * \param uq int32_t quadrature voltage
 *
 */
void FOC::InvParkClarke(int32_t ud, int32_t uq)
{
   //Inverse Park transformation
   s32fp ua = (cos * ud - sin * uq) >> CST_DIGITS;
   s32fp ub = (cos * uq + sin * ud) >> CST_DIGITS;
   //Inverse Clarke transformation
   DutyCycles[0] = ua;
   DutyCycles[1] = (-ua + FP_MUL(SQRT3, ub)) / 2;
   DutyCycles[2] = (-ua - FP_MUL(SQRT3, ub)) / 2;

   int32_t offset = SineCore::CalcSVPWMOffset(DutyCycles[0], DutyCycles[1], DutyCycles[2]);

   for (int i = 0; i < 3; i++)
   {
      /* subtract it from all 3 phases -> no difference in phase-to-phase voltage */
      DutyCycles[i] -= offset;
      /* Shift above 0 */
      DutyCycles[i] += zeroOffset;
      /* Short pulse suppression */
      if (DutyCycles[i] < minPulse)
      {
         DutyCycles[i] = 0;
      }
      else if (DutyCycles[i] > maxPulse)
      {
         DutyCycles[i] = FP_FROMINT(2);
      }
   }
}

int32_t FOC::GetMaximumModulationIndex()
{
   return modMax;
}

uint32_t FOC::sqrt(uint32_t rad)
{
   uint32_t radshift = (rad < 10000 ? 5 : (rad < 10000000 ? 9 : (rad < 1000000000 ? 13 : 15)));
   uint32_t sqrt = (rad >> radshift) + 1; //Starting value for newton iteration
   uint32_t sqrtl;

   do {
      sqrtl = sqrt;
      sqrt = (sqrt + rad / sqrt) / 2;
   } while ((sqrtl - sqrt) > 1);

   return sqrt;
}

float FOC::floatSqrt(float rad)
{
	int exp = getexp(rad);
	//Approximate start value as 2^(log2(rad) / 2 - 1) * 3
	uint32_t approx = 1 << (16 + (exp / 2 - 1)); //approximation only works from 2^-32 to 2^32
	float sqrt = approx > 0 ? 3.0f / 65536.0f * approx : 1;
	float sqrtl;
	float maxDiff = rad / 10000; //The greater the radiant, the less absolute accuracy we aim for

	if (rad <= 0) return 0;

	do {
	   sqrtl = sqrt;
	   sqrt = (sqrt + rad / sqrt) / 2;
	} while (ABS(sqrtl - sqrt) > maxDiff);

	return sqrt;
}

int FOC::getexp(float f)
{
   union
   {
      float f;
      struct {
         unsigned int mantisa : 23;
         unsigned int exponent : 8;
         unsigned int sign : 1;
      } parts;
   } floatcast = { f };
   return floatcast.parts.exponent - 127;
}
