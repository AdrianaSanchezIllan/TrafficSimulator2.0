using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Radius : MonoBehaviour
{
    private GameObject pedestrian;
    private GameObject car;

    private void OnTriggerEnter(Collider other)
    {
        
        if(other.gameObject.CompareTag("Pedestrian"))
        {
            pedestrian = other.gameObject;
        }
        if (other.gameObject.CompareTag("AutonomousVehicle"))
        {
            car = other.gameObject;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        
        if (other.gameObject.CompareTag("Pedestrian"))
        {
            pedestrian = null;
        }
        if (other.gameObject.CompareTag("AutonomousVehicle"))
        {
            car = null;
        }
    }

    public GameObject PedestrianDetected()
    {
        return pedestrian;
    }
    public bool CarInRadius(out GameObject _car)
    {
        
        if(car != null)
        {
            _car = car;
            return true;
        }
        _car = null;
        return false;
    }
}
