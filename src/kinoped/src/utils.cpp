void getTimestamp(char *buffer)
{
    char timeString[MAX_TIMESTAMP_LENGTH];
    time_t t = time(NULL);
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm = *localtime(&t);
    sprintf(buffer, "%d-%02d-%02d %02d:%02d:%02d.%06lu", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, (ts.tv_nsec / 1000));
}

/*******************************************************************************************/
/*                                                                                         */
/* void print_current_time_with_ms (void)                                                  */
/*                                                                                         */
/* Printf the current time since the epoch with milliseconds.                              */
/*                                                                                         */
/*******************************************************************************************/
void print_current_time_with_ms(void)
{
    long            ms; // Milliseconds
    time_t          s;  // Seconds
    struct timespec spec;

    clock_gettime(CLOCK_REALTIME, &spec);

    s  = spec.tv_sec;
    ms = round(spec.tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds
    if (ms > 999) {
        s++;
        ms = 0;
    }

    printf("Current time: %" PRIdMAX".%06ld seconds\n", (intmax_t)s, ms);
}

/************************************************************************************************/
/*                                                                                              */
/* double poundsToKilograms(double pounds)                                                      */
/*                                                                                              */
/* Returns kilograms for the provided pounds measurement                                        */
/*                                                                                              */
/************************************************************************************************/
double poundsToKilograms(double lbs)
{
    return (lbs * KILOGRAMS_IN_A_POUND);
}

/************************************************************************************************/
/*                                                                                              */
/* double kilogramsToPounds(double kg)                                                          */
/*                                                                                              */
/* Returns pounds for the provided kilograms measurement                                        */
/*                                                                                              */
/************************************************************************************************/
double kilogramsToPounds(double kg)
{
    return (kg / KILOGRAMS_IN_A_POUND);
}

/************************************************************************************************/
/*                                                                                              */
/* double psiToPascals(double psi)                                                              */
/*                                                                                              */
/* Returns pascals for the provided PSI measurement                                             */
/*                                                                                              */
/************************************************************************************************/
double psiToPascals(double psi)
{
    return (psi * ONE_PSI_IN_PASCALS);
}

/************************************************************************************************/
/*                                                                                              */
/* double pascalsToPSI(double pascals)                                                          */
/*                                                                                              */
/* Returns PSI for the provided pascals measurement                                             */
/*                                                                                              */
/************************************************************************************************/
double pascalsToPSI(double pascals)
{
    return (pascals / ONE_PSI_IN_PASCALS);
}

/************************************************************************************************/
/*                                                                                              */
/* double psiToBar(double psi)                                                                  */
/*                                                                                              */
/* Returns bar for the provided PSI measurement                                                 */
/*                                                                                              */
/************************************************************************************************/
double psiToBar(double psi)
{
    return (psi * ONE_PSI_IN_BAR);
}

/************************************************************************************************/
/*                                                                                              */
/* double barToPSI(double bar)                                                                  */
/*                                                                                              */
/* Returns PSI for the provided bar measurement                                                 */
/*                                                                                              */
/************************************************************************************************/
double barToPSI(double bar)
{
    return (bar / ONE_PSI_IN_BAR);
}

/************************************************************************************************/
/*                                                                                              */
/* double barToPascals(double bar)                                                              */
/*                                                                                              */
/* Returns pascals for the provided bar measurement                                             */
/*                                                                                              */
/************************************************************************************************/
double barToPascals(double bar)
{
    return (bar * ONE_BAR_IN_PASCALS);
}

/************************************************************************************************/
/*                                                                                              */
/* double pascalsToBar(double pascals)                                                          */
/*                                                                                              */
/* Returns bar for the provided pascals measurement                                             */
/*                                                                                              */
/************************************************************************************************/
double pascalsToBar(double pascals)
{
    return (pascals / ONE_BAR_IN_PASCALS);
}


/************************************************************************************************/
/*                                                                                              */
/* double metersToInches(double meters)                                                         */
/*                                                                                              */
/* Returns inches for the provided meters measurement                                           */
/*                                                                                              */
/************************************************************************************************/
double metersToInches(double meters)
{
    return (meters * INCHES_IN_A_METER);
}

/************************************************************************************************/
/*                                                                                              */
/* double inchesToMeters(double inches)                                                         */
/*                                                                                              */
/* Returns meters for the provided inches measurement                                           */
/*                                                                                              */
/************************************************************************************************/
double inchesToMeters(double inches)
{
    return (inches / INCHES_IN_A_METER);
}

/************************************************************************************************/
/*                                                                                              */
/* double mmToInches(double mm)                                                                 */
/*                                                                                              */
/* Returns inches for the provided millimeter measurement                                       */
/*                                                                                              */
/************************************************************************************************/
double mmToInches(double mm)
{
   return (mm / MM_IN_AN_INCH);
}

/************************************************************************************************/
/*                                                                                              */
/* double inchesToMM(double inches)                                                             */
/*                                                                                              */
/* Returns millimeters for the provided inch measurement                                        */
/*                                                                                              */
/************************************************************************************************/
double inchesToMM(double inches)
{
    return (inches * MM_IN_AN_INCH);
}

/************************************************************************************************/
/*                                                                                              */
/* double getRadiansFromPulseCount(unsigned int pulse_count, unsigned int count_per_revolution) */
/*                                                                                              */
/* Returns radians calculated from the pulse count and counts per revolution.                   */
/*                                                                                              */
/************************************************************************************************/
double getRadiansFromPulseCount(unsigned int pulse_count, unsigned int count_per_revolution)
{
   double pulses_per_radian = (double)count_per_revolution / TWO_PI;
   if (count_per_revolution)
   {
      return ((double)pulse_count / pulses_per_radian);
   }
   else
   {
      return (-1.0);
   }
}

/************************************************************************************************/
/*                                                                                              */
/* double getPulseCountFromRadians(double radians, unsigned int count_per_revolution)           */
/*                                                                                              */
/* Returns pulse count calculated from the radians and counts per revolution.                   */
/*                                                                                              */
/************************************************************************************************/
double getPulseCountFromRadians(double radians, unsigned int count_per_revolution)
{
   // Pulses per Degree = Number of encoder pulses per rotation/Number of degrees in a circle
   double pulses_per_radian = (double)count_per_revolution / TWO_PI;
   if (count_per_revolution)
   {
      return (double)(radians * pulses_per_radian);
   }
   else
   {
      return (-1.0);
   }
}

/************************************************************************************************/
/*                                                                                              */
/* double getDegreesFromPulseCount(unsigned int pulse_count, unsigned int count_per_revolution) */
/*                                                                                              */
/* Returns degrees calculated from the pulse count and counts per revolution.                   */
/*                                                                                              */
/************************************************************************************************/
double getDegreesFromPulseCount(unsigned int pulse_count, unsigned int count_per_revolution)
{
   double pulses_per_degree = (double)count_per_revolution / DEGREES_IN_A_CIRCLE;
   if (count_per_revolution)
   {
      return ((double)pulse_count / pulses_per_degree);
   }
   else
   {
      return (-1.0);
   }
}

/************************************************************************************************/
/*                                                                                              */
/* double getPulseCountFromDegrees(double degrees, unsigned int count_per_revolution)           */
/*                                                                                              */
/* Returns pulse count calculated from the degrees and counts per revolution.                   */
/*                                                                                              */
/************************************************************************************************/
double getPulseCountFromDegrees(double degrees, unsigned int count_per_revolution)
{
   // Pulses per Degree = Number of encoder pulses per rotation/Number of degrees in a circle
   double pulses_per_degree = (double)count_per_revolution / DEGREES_IN_A_CIRCLE;
   if (count_per_revolution)
   {
      return (double)(degrees * pulses_per_degree);
   }
   else
   {
      return (-1.0);
   }
}

/************************************************************************************************/
/*                                                                                              */
/* double degreesToRadians(double degrees)                                                      */
/*                                                                                              */
/* Returns radians calculated from degrees                                                      */
/*                                                                                              */
/************************************************************************************************/
double degreesToRadians(double degrees)
{
   return ((degrees * PI) / DEGREES_IN_HALF_OF_A_CIRCLE);
}

/************************************************************************************************/
/*                                                                                              */
/* double radiansToDegrees(double radians)                                                      */
/*                                                                                              */
/* Returns degrees calculated from radians                                                      */
/*                                                                                              */
/************************************************************************************************/
double radiansToDegrees(double radians)
{
   return ((radians * DEGREES_IN_HALF_OF_A_CIRCLE) / PI);
}


/************************************************************************************************/
/*                                                                                              */
/* double randfrom(double minV, double maxV)                                                    */
/*                                                                                              */
/* Returns a random number between the min and max                                              */
/*                                                                                              */
/************************************************************************************************/
double randfrom(double minV, double maxV) 
{
    double range = (maxV - minV); 
    double div = RAND_MAX / range;
    return minV + (rand() / div);
}
