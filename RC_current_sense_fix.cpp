   // Calculate the impedance of the RC circuit formed by the resistor and the capacitor across the current sense resistor
   float angularFrequency = 2 * PI * transitionFrequency; // Angular frequency in radians per second
   float rcImpedance = sqrt(coilResistance * coilResistance + (1 / (angularFrequency * CAPACITOR)) * (1 / (angularFrequency * CAPACITOR))); // Impedance of the RC circuit in ohms
