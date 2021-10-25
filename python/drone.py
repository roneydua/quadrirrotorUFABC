#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May  6 08:06:32 2021
@author: roney
"""
import sympy as sp
import numpy as np
import funcoesQuaternion as fq

grav = np.array([0.0, 0.0, 9.81])
# momentos de inercia
Ix = 16.83e-3
Iy = 16.83e-3
Iz = 28.34e-3
# constantes dos motores
kf = 1.4351e-5
km = 2.4086e-7
# paramentros geometricos do quadrirrotor
b = 0.26
m = 1.03
Ir = 5.0e-5
# tempo de assentamento do da dinamica do motor
ta = 0.03
DA = 0.5

# %%Classe drone SDRE discreto

# Classe drone discreto
# definiçẽos básicas


class Drone:
    """
    construtor
    @param self The object pointer.
    """

    def __init__(self, *args, **kwargs):
        """The constructor"""

        ## Momento de inercia Ix
        self.Ix = Ix
        ## Momento de inercia Iy
        self.Iy = Iy
        ## Momento de inercia Iz
        self.Iz = Iz
        ## Constante de força
        self.kf = kf
        self.km = km
        self.b = b
        self.grav = grav
        self.m = m
        # limites dos motores rad/s
        self.omegaSquareMin = 296.0 ** 2.0
        self.omegaSquareMax = 657.0 ** 2.0
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def updateStates(self):
        """Returna o conjugado"""
        return fq.conj(self.q)
