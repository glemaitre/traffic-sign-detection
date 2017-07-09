/*
By downloading, copying, installing or using the software you agree to this license.
If you do not agree to this license, do not download, install,
copy or use the software.


                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2015, 
      Guillaume Lemaitre (g.lemaitre58@gmail.com),
      Johan Massich (mailsik@gmail.com),
      Gerard Bahi (zomeck@gmail.com),
      Yohan Fougerolle (Yohan.Fougerolle@u-bourgogne.fr).
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are disclaimed.
In no event shall copyright holders or contributors be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

// random-standalone.cpp (Autonome/Standalone)
// Implementation de la classe Random / Implementation of the class Random
#include "random-standalone.h"

#include <cfloat> //DBL_EPSILON

inline static double sqr(double x) {return x*x;}

//Ces constantes sont requises par l'algorithme du generateur aleatoire
//These consts are required by the algorithm
const int Random::NTAB = 32;

const long Random::IM1 = 2147483563;
const long Random::IM2 = 2147483399;
const long Random::IMM1 = Random::IM1 - 1;
const double Random::AM = 1.0/Random::IM1;
const int Random::IA1 = 40014;
const int Random::IA2 = 40692;
const int Random::IQ1 = 53668;
const int Random::IQ2 = 52774;
const int Random::IR1 = 12211;
const int Random::IR2 = 3791;
const int Random::NDIV = 1 + Random::IMM1/Random::NTAB;
const double Random::EPS = DBL_EPSILON;
const double Random::RNMX = 1.0-Random::EPS;

Random::Random(long seed)
    :iv(0)
{
    iv = new long[Random::NTAB];
    idum   = 0;
    idum2  = 123456789L;
    iy     = 0;
    randomize(seed);
}

void Random::randomize(long seed)
{
    idum = (seed <= 0) ? (seed == 0 ? 1 : -seed) : seed; //be sure to prevent idum=0

    idum2 = idum;
    if (idum2 < 0) idum2 = -idum2;

    long k = 0;
    for(int j = NTAB+7 ; j >= 0 ; j--)
    {
        k = idum/IQ1;
        idum = IA1*(idum - k*IQ1) - k*IR1;
        if (idum < 0)
            idum += IM1;

        if (j < NTAB)
            iv[j] = idum;
    }

    iy = iv[0];
}
//fin randomize()

//Cette fonction renvoie un double pseudo-aleatoire uniformement dans ]0;1[
//Il s'agit de l'algorithme de L'Ecuyer avec melange de Bays-Durham
//This function returns a double, taken uniformly in ]0;1[
//It is the algorithm of L'Ecuyer with a Bays-Durham shuffle
double Random::theRandom(void)
{
    long k = idum/IQ1;
    idum = IA1*(idum - k*IQ1) - k*IR1;
    if (idum < 0)
        idum +=IM1;

    k = idum2/IQ2;
    idum2 = IA2*(idum2 -k*IQ2) - k*IR2;
    if (idum2 < 0)
        idum2 +=IM2;

    long j = iy/NDIV;
    iy = iv[j] - idum2;
    iv[j] = idum;
    if (iy < 1)
        iy += IMM1;

    double temp = AM*iy;
    if (temp >= RNMX)
        return RNMX;  //empeche de renvoyer 1 / prevents from returning 1
    else
        return temp;
}
//fin theRandom()

//Renvoie un double de la Gaussienne de moyenne et d'ecartype specifies
//Returns a double taken on a Gaussian with specified mean and standard deviation
double Random::gaussian(double mean, double standardDeviation)
{
    const int NbTirages = 12; //augmenter pour une meilleure precision. 12 est bien.
    //increase for better precision. 12 works fine.
    double valeur = 0;
    for(int i=0 ; i < NbTirages ; ++i)
        valeur += uniform();

    //on recentre la somme / centering the sum
    valeur -= NbTirages/2;

    //on etale suivant l'ecartype / spread with standard deviation
    //le 12 n'a rien a voir avec NbTirages, mais explique pourquoi justement, on prend souvent
    //NbTirages = 12
    //the 12 is not related to NbTirages, but it explains why it is often chosen that NbTirages=12
    valeur *= (NbTirages == 12) ? standardDeviation
                                : sqrt(12/static_cast<double>(NbTirages))*standardDeviation;

    //on centre sur la moyenne / debias
    valeur += mean;

    return valeur;
}
//fin gaussian()

//fin random-standalone.cpp
