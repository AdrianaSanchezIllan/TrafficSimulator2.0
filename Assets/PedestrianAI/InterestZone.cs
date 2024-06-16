using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InterestZone : MonoBehaviour
{
    public List<InterestLocation> locations = new List<InterestLocation>();
    public string interestAction; // Acci�n que se realiza en esta zona
    public GameObject actionIndicatorPrefab; // Prefab para mostrar la acci�n actual

    private void Start()
    {
        locations.Clear();
        foreach (Transform child in transform)
        {
            InterestLocation location = child.GetComponent<InterestLocation>();
            if (location != null)
            {
                locations.Add(location);
            }
            else
            {
                Debug.LogWarning("El hijo " + child.name + " no tiene el componente InterestLocation.");
            }
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        PedestrianBehaviour pedestrianBehaviour = other.GetComponent<PedestrianBehaviour>();
        if (pedestrianBehaviour != null)
        {
            pedestrianBehaviour.currentZone = this;
            pedestrianBehaviour.currentAction = this.interestAction;
            //Debug.Log("Entered interest zone: " + interestAction);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        PedestrianBehaviour pedestrianBehaviour = other.GetComponent<PedestrianBehaviour>();
        if (pedestrianBehaviour != null)
        {
            pedestrianBehaviour.currentZone = null;
            Debug.Log("Exited interest zone");
        }
    }

    // M�todo para obtener la primera localizaci�n libre
    public InterestLocation GetFirstFreeLocation()
    {
        foreach (var location in locations)
        {
            if (!location.IsOccupied)
            {
                location.Occupy();
                return location;
            }
        }
        return null; // Si no hay localizaciones libres, devuelve null
    }

    // M�todo para liberar una localizaci�n
    public void VacateLocation(InterestLocation location)
    {
        if (locations.Contains(location))
        {
            location.Vacate();
        }
    }

    // M�todo para obtener la localizaci�n libre m�s cercana
    public InterestLocation GetClosestLocation(Transform pedestrianTransform)
    {
        InterestLocation closestLocation = null;
        float closestDistance = float.MaxValue;

        foreach (var location in locations)
        {
            float distance = Vector3.Distance(pedestrianTransform.position, location.transform.position);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestLocation = location;
            }
        }


        return closestLocation;
    }

    // M�todo para obtener la localizaci�n libre m�s lejana
    public InterestLocation GetFurthestLocation(Transform pedestrianTransform)
    {
        InterestLocation furthestLocation = null;
        float furthestDistance = float.MinValue;

        foreach (var location in locations)
        {
            float distance = Vector3.Distance(pedestrianTransform.position, location.transform.position);
            if (distance > furthestDistance)
            {
                furthestDistance = distance;
                furthestLocation = location;
            }
        }

        return furthestLocation;
    }
}