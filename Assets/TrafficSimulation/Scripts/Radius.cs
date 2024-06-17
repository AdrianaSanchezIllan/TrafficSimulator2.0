using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Radius : MonoBehaviour
{
    private GameObject carNeighbor;

    private void OnTriggerEnter(Collider other)
    {
        
        if(other.gameObject.CompareTag("AutonomousVehicle"))
        {
            carNeighbor = other.gameObject;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        
        if (other.gameObject.CompareTag("AutonomousVehicle"))
        {
            carNeighbor = null;
        }
    }

    public GameObject CarDetected()
    {
        return carNeighbor;
    }
}
