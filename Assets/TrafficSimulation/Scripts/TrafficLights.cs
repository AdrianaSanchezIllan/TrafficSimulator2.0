using System.Collections;
using System.Collections.Generic;
using System.Data;
using TrafficSimulation;
using UnityEngine;

public class TrafficLights : MonoBehaviour
{
    public int lightGroupId;  // Belong to traffic light 1 or 2?
    public Intersection intersection;
    
    public bool greenForPedestrian = false;
    Light pointLight;

    void Start()
    {
        pointLight = this.transform.GetChild(0).GetComponent<Light>();
        SetTrafficLightColor();
    }

    // Update is called once per frame
    void Update()
    {
        SetTrafficLightColor();
    }

    void SetTrafficLightColor()
    {
        if (lightGroupId == intersection.currentRedLightsGroup)
            pointLight.color = new Color(1, 0, 0); // Red
        else
            pointLight.color = new Color(0, 1, 0); // Green
    }
    public void IsRedForCars()
    {
        if( pointLight.color == new Color(1, 0, 0))
        {
            greenForPedestrian = true;
        }
    }

    public float TimeToChange()
    {
        return intersection.lightsDuration - intersection.timeSinceChange;
    }
}
