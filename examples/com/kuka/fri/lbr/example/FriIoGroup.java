package com.kuka.fri.lbr.example;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.io.IIoDefinition;
import com.kuka.io.IIoDefinitionProvider;
import com.kuka.io.IIoValueProvider;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>FRI</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * ./.
 */
@Singleton
public class FriIoGroup
{
    /** The name of the IO group as it is configured in WorkVisual. */
    public static final String NAME = "FRI";

    private IIoDefinitionProvider definitionProvider;
    private IIoValueProvider valueProvider;

    /**
     * Creates a new instance.
     * 
     * @param ioDefinitionProvider
     *            the provider for accessing IO definitions
     * @param ioValueProvider
     *            the provider for accessing IO values
     */
    @Inject
    public FriIoGroup(IIoDefinitionProvider ioDefinitionProvider,
            IIoValueProvider ioValueProvider)
    {
        definitionProvider = ioDefinitionProvider;
        valueProvider = ioValueProvider;
    }

    /**
     * Gets the value of the <b>digital input '<i>In_Bool_Clock_Enabled</i>'</b>.<br>
     * <i>This method is automatically generated. Please, do not modify!</i>
     * <p>
     * <b>I/O direction and type:</b><br>
     * digital input
     * <p>
     * <b>User description of the I/O:</b><br>
     * ./.
     * <p>
     * <b>Range of the I/O value:</b><br>
     * [false; true]
     *
     * @return current value of the digital input 'In_Bool_Clock_Enabled'
     */
    public boolean getIn_Bool_Clock_Enabled()
    {
        IIoDefinition definition = definitionProvider.getIoDefinition(NAME, "In_Bool_Clock_Enabled");
        return valueProvider.getIoValue(definition).getValueAsBoolean();
    }

    /**
     * Gets the value of the <b>digital output '<i>Out_Bool_Enable_Clock</i>'</b>.<br>
     * <i>This method is automatically generated. Please, do not modify!</i>
     * <p>
     * <b>I/O direction and type:</b><br>
     * digital output
     * <p>
     * <b>User description of the I/O:</b><br>
     * ./.
     * <p>
     * <b>Range of the I/O value:</b><br>
     * [false; true]
     *
     * @return current value of the digital output 'Out_Bool_Enable_Clock'
     */
    public boolean getOut_Bool_Enable_Clock()
    {
        IIoDefinition definition = definitionProvider.getIoDefinition(NAME, "Out_Bool_Enable_Clock");
        return valueProvider.getIoValue(definition).getValueAsBoolean();
    }

    /**
     * Sets the value of the <b>digital output '<i>Out_Bool_Enable_Clock</i>'</b>.<br>
     * <i>This method is automatically generated. Please, do not modify!</i>
     * <p>
     * <b>I/O direction and type:</b><br>
     * digital output
     * <p>
     * <b>User description of the I/O:</b><br>
     * ./.
     * <p>
     * <b>Range of the I/O value:</b><br>
     * [false; true]
     *
     * @param value
     *            the value, which has to be written to the digital output 'Out_Bool_Enable_Clock'
     */
    public void setOut_Bool_Enable_Clock(java.lang.Boolean value)
    {
        IIoDefinition definition = definitionProvider.getIoDefinition(NAME, "Out_Bool_Enable_Clock");
        valueProvider.setValue(definition, value);
    }

    /**
     * Gets the value of the <b>digital output '<i>Out_Integer_Seconds</i>'</b>.<br>
     * <i>This method is automatically generated. Please, do not modify!</i>
     * <p>
     * <b>I/O direction and type:</b><br>
     * digital output
     * <p>
     * <b>User description of the I/O:</b><br>
     * ./.
     * <p>
     * <b>Range of the I/O value:</b><br>
     * [0; 4294967295]
     *
     * @return current value of the digital output 'Out_Integer_Seconds'
     */
    public java.lang.Long getOut_Integer_Seconds()
    {
        IIoDefinition definition = definitionProvider.getIoDefinition(NAME, "Out_Integer_Seconds");
        return valueProvider.getIoValue(definition).getValue().longValue();
    }

    /**
     * Sets the value of the <b>digital output '<i>Out_Integer_Seconds</i>'</b>.<br>
     * <i>This method is automatically generated. Please, do not modify!</i>
     * <p>
     * <b>I/O direction and type:</b><br>
     * digital output
     * <p>
     * <b>User description of the I/O:</b><br>
     * ./.
     * <p>
     * <b>Range of the I/O value:</b><br>
     * [0; 4294967295]
     *
     * @param value
     *            the value, which has to be written to the digital output 'Out_Integer_Seconds'
     */
    public void setOut_Integer_Seconds(java.lang.Long value)
    {
        IIoDefinition definition = definitionProvider.getIoDefinition(NAME, "Out_Integer_Seconds");
        valueProvider.setValue(definition, value);
    }

    /**
     * Gets the value of the <b>analog output '<i>Out_Analog_Deci_Seconds</i>'</b>.<br>
     * <i>This method is automatically generated. Please, do not modify!</i>
     * <p>
     * <b>I/O direction and type:</b><br>
     * analog output
     * <p>
     * <b>User description of the I/O:</b><br>
     * ./.
     * <p>
     * <b>Range of the I/O value:</b><br>
     * [0.0; 1.0]
     *
     * @return current value of the analog output 'Out_Analog_Deci_Seconds'
     */
    public double getOut_Analog_Deci_Seconds()
    {
        IIoDefinition definition = definitionProvider.getIoDefinition(NAME, "Out_Analog_Deci_Seconds");
        return valueProvider.getIoValue(definition).getValue().doubleValue();
    }

    /**
     * Sets the value of the <b>analog output '<i>Out_Analog_Deci_Seconds</i>'</b>.<br>
     * <i>This method is automatically generated. Please, do not modify!</i>
     * <p>
     * <b>I/O direction and type:</b><br>
     * analog output
     * <p>
     * <b>User description of the I/O:</b><br>
     * ./.
     * <p>
     * <b>Range of the I/O value:</b><br>
     * [0.0; 1.0]
     *
     * @param value
     *            the value, which has to be written to the analog output 'Out_Analog_Deci_Seconds'
     */
    public void setOut_Analog_Deci_Seconds(double value)
    {
        IIoDefinition definition = definitionProvider.getIoDefinition(NAME, "Out_Analog_Deci_Seconds");
        valueProvider.setValue(definition, value);
    }

}
