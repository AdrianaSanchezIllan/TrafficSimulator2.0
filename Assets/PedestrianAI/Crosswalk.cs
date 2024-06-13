using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Crosswalk : MonoBehaviour
{
    public Collider collider; // Collider del paso de cebra
    public TrafficLights trafficLight; // Semáforo asociado
    //public float detectionRadius = 1f; // Radio de detección del paso de cebra

    private void Awake()
    {
        //se asignan manualmente en el inspector
        //collider = GetComponent<Collider>();
        //trafficLight = GetComponentInChildren<TrafficLights>();

        if (collider == null)
        {
            Debug.LogError("Collider not found on Crosswalk!");
        }

        if (trafficLight == null)
        {
            Debug.LogError("TrafficLights not found on Crosswalk!");
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        PedestrianBehaviour pedestrian = other.GetComponent<PedestrianBehaviour>();
        if (pedestrian != null)
        {
            pedestrian.EnterCrosswalkArea(this);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        PedestrianBehaviour pedestrian = other.GetComponent<PedestrianBehaviour>();
        if (pedestrian != null)
        {
            pedestrian.ExitCrosswalkArea(this);
        }
    }
}
