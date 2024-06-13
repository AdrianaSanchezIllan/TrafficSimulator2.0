using UnityEngine;
using UnityEngine.AI;
using System.Collections;
using BehaviourAPI.Core;
using BehaviourAPI.StateMachines;
using BehaviourAPI.BehaviourTrees;
using System.Collections.Generic;
using static PedestrianBehaviour;
using static UnityEditor.ShaderData;
using System.Runtime.ConstrainedExecution;

public class PedestrianBehaviour : MonoBehaviour
{
    public NavMeshAgent agent;
    public float range; // Radio de deambulación
    public Transform centrePoint; // Centro del área de deambulación

    private List<Crosswalk> crosswalks = new List<Crosswalk>(); // Lista de pasos de cebra
    private Crosswalk nearbyCrosswalk; // Variable para almacenar el paso de cebra cercano
    private bool hasCrossed = false;
    private Vector3 originalDestination; // Para almacenar el destino original antes de cruzar
    private bool canCrossNow = false;
    private bool isInCrosswalk = false; // Flag para controlar la entrada y salida
                                        //public TrafficLights trafficLight; // Referencia al semáforo


    private Animator animator;
    private static readonly int IsWalking = Animator.StringToHash("isWalking");

    private GameObject currentActionIndicator;
    public InterestZone currentZone;
    public Transform currentLocation;
    public string currentAction;
    private bool isActionCompleted = false;

    Transform location;
    private bool inTrafficLight = false;
    private bool isCrossingStreet = false;
    
    private void Awake()
    {
        agent = GetComponent<NavMeshAgent>();
        animator = GetComponent<Animator>();

        if (animator == null)
        {
            Debug.LogError("Animator not found!");
        }

        if (agent == null)
        {
            agent = gameObject.AddComponent<NavMeshAgent>();
        }

        if (centrePoint == null)
        {
            centrePoint = transform;
        }

        // Buscar todos los objetos con la etiqueta "Crosswalk" y añadir sus componentes a la lista
        GameObject[] crosswalkObjects = GameObject.FindGameObjectsWithTag("Crosswalk");
        foreach (var crosswalkObject in crosswalkObjects)
        {
            Crosswalk crosswalk = crosswalkObject.GetComponent<Crosswalk>();
            if (crosswalk != null)
            {
                crosswalks.Add(crosswalk);
            }
        }
    }

    // Estado de deambulación
    public void StartRoaming()
    {
        StartCoroutine(Roam());
    }

    private IEnumerator Roam()
    {
        while (true)
        {
            if (agent.remainingDistance <= agent.stoppingDistance)
            {
                Vector3 point;
                if (RandomPoint(centrePoint.position, range, out point))
                {
                    Debug.DrawRay(point, Vector3.up, Color.blue, 1.0f);
                    agent.SetDestination(point);
                }
            }

            bool isMoving = agent.velocity.magnitude > 0.1f;
            animator.SetBool(IsWalking, isMoving);
            yield return null;

        }
    }

    // Método para verificar si el peatón en algún paso de cebra
    private bool InCrosswalkArea()
    {
        if(nearbyCrosswalk != null)
        {
            Debug.Log("en paso de cebra");
            return true;
        }
        nearbyCrosswalk = null;
        return false;
    }

    //guarda el paso de cebra en el q esta
    public void EnterCrosswalkArea(Crosswalk crosswalk)
    {
        if (!isInCrosswalk) // Verificar si ya está en el paso de cebra
        {
            isInCrosswalk = true;
            //agent.isStopped = true;
            nearbyCrosswalk = crosswalk;
            Debug.Log("Entered crosswalk area" + nearbyCrosswalk);
        }
    }

    public void ExitCrosswalkArea(Crosswalk crosswalk)
    {
        if (isInCrosswalk && crosswalk == nearbyCrosswalk)
        {
            isInCrosswalk = false;
            nearbyCrosswalk = null;
            Debug.Log("Exited crosswalk area");
        }
    }

    // Método para mover al peatón al inicio del paso de cebra
    public void MoveToCrosswalk()
    {
        originalDestination = agent.destination; // Guardar el destino original
        agent.SetDestination(GetCrosswalkEntrance()); // Mover al inicio del paso de cebra
        StartCoroutine(WaitAtCrosswalk());
    }

    // Obtener el punto de entrada del paso de cebra
    private Vector3 GetCrosswalkEntrance()
    {
        return nearbyCrosswalk.collider.ClosestPoint(agent.transform.position);
    }

    // Estado para esperar en el paso de cebra
    private IEnumerator WaitAtCrosswalk()
    {
        while (true)
        {
            if (agent.remainingDistance <= agent.stoppingDistance)
            {
                agent.isStopped = true; // Detener al agente
                if (nearbyCrosswalk.trafficLight.greenForPedestrian)
                {
                    agent.isStopped = false; // Reanudar el agente
                    canCrossNow = true;
                    yield break;
                }
            }

            yield return new WaitForSeconds(1.0f); // Esperar antes de verificar nuevamente
        }
    }
    private bool CanCrossNow()
    {
        return canCrossNow;
    }
    // Estado para comenzar a cruzar la calle
    public void StartCrossing()
    {
        StartCoroutine(CrossStreet());
    }

    private IEnumerator CrossStreet()
    {
        while (true)
        {
            // Verificar si el semáforo del crosswalk cercano está en verde
            if (nearbyCrosswalk != null && nearbyCrosswalk.trafficLight.greenForPedestrian)
            {
                agent.SetDestination(originalDestination); // Continuar al destino original
                hasCrossed = true;
                yield break;
            }

            yield return new WaitForSeconds(1.0f); // Esperar antes de verificar nuevamente
        }
    }

    public bool CheckIfStreetCrossed()
    {
        return hasCrossed;
    }



    private bool RandomPoint(Vector3 center, float range, out Vector3 result)
    {
        Vector3 randomPoint = center + Random.insideUnitSphere * range;
        NavMeshHit hit;
        if (NavMesh.SamplePosition(randomPoint, out hit, 1.0f, NavMesh.AllAreas))
        {
            result = hit.position;
            return true;
        }

        result = Vector3.zero;
        return false;
    }

    public void MoveTo(Vector3 destination)
    {
        agent.SetDestination(destination);
        StartCoroutine(CheckIfArrived());
    }

    private IEnumerator CheckIfArrived()
    {
        while (true)
        {
            if (agent.remainingDistance <= agent.stoppingDistance)
            {
                agent.isStopped = true;
                yield break; // Terminar coroutine cuando el agente llega al destino
            }

            bool isMoving = agent.velocity.magnitude > 0.1f;
            animator.SetBool(IsWalking, isMoving);

            yield return null;
        }
    }
    /*
private IEnumerator WaitForGreenLight()
{
    while (!trafficLight.IsRedForCars())
    {
        yield return new WaitForSeconds(0.5f); // Esperar 0.5 segundos antes de verificar nuevamente
    }

    isCrossingStreet = true;
    StartRoaming();
}
*/

    // Acciones en zonas de interés
    public void PerformAction(string action)
    {
        switch (action)
        {
            case "Coffee":
                Debug.Log("Tomando un café.");
                break;
            case "Park":
                Debug.Log("Dando una vuelta corriendo.");
                break;
            case "Supermarket":
                Debug.Log("Comprando víveres.");
                break;
            default:
                Debug.LogWarning("Acción desconocida: " + action);
                break;
        }
    }

    public Status EnterInterestZone()
    {
        if (currentZone != null)
        {
            currentLocation = currentZone.GetAvailableLocation();
            Debug.Log("se asigna currentLocation");
            if (currentLocation != null)
            {
                return Status.Success;
            }
        }
        return Status.Failure;
    }

    /*public Status MoveToLocation()
    {
        if (currentLocation != null)
        {
            Debug.Log(currentLocation);
            MoveTo(currentLocation.position);
            if (isPathComplete())
            {
                Debug.Log("he llegado a la silla");
                return Status.Success;
            }
            return Status.Running;
        }
        return Status.Failure;
       
    }*/
    public Status MoveToLocation()
    {
        if (currentLocation != null)
        {
            Debug.Log($"Moving to location: {currentLocation.position}");
            MoveTo(currentLocation.position);

            if (isPathComplete())
            {
                Debug.Log("Path is complete.");
                return Status.Success;
            }
            else
            {
                Debug.Log("Moving to location...");
                return Status.Running;
            }
        }
        return Status.Failure;
    }
    public Status PerformActionBT()
    {
        if (!isActionCompleted)
        {
            isActionCompleted = false;
            StartCoroutine(PerformInterestAction());
            return Status.Running;
        }
        return Status.Success;
    }

    private IEnumerator PerformInterestAction()
    {
        while (agent.remainingDistance > agent.stoppingDistance)
        {
            yield return null;
        }

        agent.isStopped = true;
        PerformAction(currentZone.interestAction);

        if (currentActionIndicator != null)
        {
            Destroy(currentActionIndicator);
        }

        if (currentZone.actionIndicatorPrefab != null)
        {
            currentActionIndicator = Instantiate(currentZone.actionIndicatorPrefab, transform);
            // Ajustar la posición de la imagen por encima del personaje
            currentActionIndicator.transform.localPosition = new Vector3(0, 2.0f, 0);
        }

        // Simular duración de la acción
        yield return new WaitForSeconds(10.0f); // Por ejemplo,10 segundos para tomar café

        if (currentActionIndicator != null)
        {
            Destroy(currentActionIndicator);
        }

        agent.isStopped = false;
        currentZone.VacateLocation(currentLocation); // Liberar la ubicación
        currentLocation = null;
        isActionCompleted = true;
    }

    

    #region Behaviour Tree Nodes

    public Status CheckIfCanPerformAction()
    {
        return currentZone != null ? Status.Success : Status.Failure;
    }

    

    public Status IsInCoffeeShop()
    {
        if (currentAction == "Coffee")
        {
            Debug.Log("es una coffee shop");
            return Status.Success;
        }
        return Status.Failure;
    }
    public Status IsInPark()
    {
        if (currentAction == "Park")
        {
            return Status.Success;
        }
        return Status.Failure;
    }

    public Status IsInSupermarket()
    {
        if (currentAction == "Supermarket")
        {
            return Status.Success;
        }
        return Status.Failure;
    }

    #endregion

    public bool IsInterestZone()
    {
        return currentZone != null;
    }

    public bool HasCompletedAction()
    {
        return isActionCompleted;
    }


    public bool isPathComplete()
    {
        return (!agent.pathPending &&
                agent.remainingDistance <= agent.stoppingDistance &&
                (!agent.hasPath || agent.velocity.sqrMagnitude == 0f));
    }

    public Status PathStatus()
    {
        if (!agent.pathPending &&
            agent.remainingDistance <= agent.stoppingDistance &&
            (!agent.hasPath || agent.velocity.sqrMagnitude == 0f))
        {
            return Status.Success;
        }
        return Status.Running;
    }

    public Status ActionHasBeenCompleted()
    {
        if(isActionCompleted)
        {
            return Status.Success;
        }
        return Status.Running;
    }

}
