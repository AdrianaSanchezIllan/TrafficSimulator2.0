using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Radar : MonoBehaviour
{
    private GameObject car;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnTriggerEnter(Collider other)
    {

        if (other.gameObject.CompareTag("AutonomousVehicle"))
        {
            car = other.gameObject;
        }
    }

    private void OnTriggerExit(Collider other)
    {

        if (other.gameObject.CompareTag("AutonomousVehicle"))
        {
            car = null;
        }
    }

    public GameObject RadarActive()
    {
        return car;
    }
}
