BCP2 - BCP Outline for how constants are defined in code

All constants are defined in frc.robot.constants as a class pertaining to whatever category needed
In the class's class scope, all constants should be defined as static variables for use in actual code

See Constants/ElevatorConstants.java as an example
If constants are different in real situations vs a simulation sim,  add `{constant}Sim`. constants 
If changing any names of the constants, remember to use the refractoring functions provided in your IDE

```java
public class MechanismsConstants {
    public static double exampleConstant1;
    public static double exampleConstant2;
    public static double exampleConstant1Sim;
    public static double exampleConstant2Sim;
}
```