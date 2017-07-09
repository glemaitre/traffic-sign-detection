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
#pragma once

#include <cmath>
#include <limits>

class Random //Autonome/Standalone
{
private:
    template<typename T>
    static inline bool typeIsInteger(void)
    {return std::numeric_limits<T>::is_integer;}
    //Si le numeric_limits<T> ne marche pas/if numeric_limits doesn't work
    //{return static_cast<T>(1)/static_cast<T>(2)==static_cast<T>(0);}

public:
    Random(long seed=0);
    ~Random() {if (iv) delete [] iv; iv = 0;}

    //Reinitialise la graine / Inits the seed
    //Attention, 0 et 1 pour la graine donnent la meme suite
    //Beware that 0 and 1 for the seed give the same sequence
    void randomize(long thatSeed=0);

    //Pour un type T, renvoie une valeur uniformement dans [min;max]
    //(min et max exclu pour les types non entiers)
    //For a type T, returns a value uniformly in [min;max]
    //(min and max excluded for non integral types)
    template <typename T>
    inline T uniform(T min, T max)
    {return static_cast<T>(min+(max+(typeIsInteger<T>()?1:0)-min)*theRandom());}
    
    //Par defaut : uniform<double>(0,1) / default : uniform<double>(0,1)
    inline double uniform(void) {return theRandom();}

    //Renvoie un nombre selon la Gaussienne de moyenne et d'ecartype specifies
    //Par defaut, c'est la loi normale centree reduite
    //Returns a double taken on a Gaussian with specified mean and standard dev.
    //By default, it is the Normal law with mean=0, std dev=1
    double gaussian(double mean=0, double standardDeviation=1);

    //Exponential
    inline double exponential(double lambda)
    {return -std::log(uniform())/lambda;}

private://methodes "interdites" / "forbidden" methods
    Random(const Random&) {};
    Random operator=(const Random&) {return *this;}

private:
    //Toutes ces constantes sont definies pour l'algorithme du generateur
    //Useful consts
    static const long int IM1;
    static const long int IM2;
    static const long int IMM1;

    static const double AM;

    static const int IA1;
    static const int IA2;
    static const int IQ1;
    static const int IQ2;
    static const int IR1;
    static const int IR2;

    static const int NDIV;

    static const double EPS;
    static const double RNMX;

private:
    //Ces variables sont utilisees pour les calculs du generateur
    //Useful variables
    long idum;
    long idum2;
    long iy;
    static const int NTAB;
    long* iv;

    //Cette fonction renvoie un double aleatoire uniforme dans ]0;1[
    //C'est le coeur du generateur
    //The kernel of the generator : returns a double uniformly in ]0;1[
    double theRandom(void);
};
