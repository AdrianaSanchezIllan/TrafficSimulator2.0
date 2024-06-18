using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class RandomPedestrianSpawner : MonoBehaviour
{
    public GameObject[] pedestrianPrefabs; // Array que contiene los prefabs de los peatones disponibles
    public int numberOfPedestrians; // N�mero de peatones a aparecer
    public float spawnRadius; // Radio dentro del cual aparecer�n los peatones
    private int numHooligans = 7;//Numero de holligans debe ser multiplo de 6 (mas o menos)

    void Start()
    {
        SpawnPedestrians();
        SpawnHooligans();
    }

    void SpawnPedestrians()
    {
        for (int i = 0; i < numberOfPedestrians; i++)
        {
            // Selecciona un prefab de peat�n aleatorio del array
            GameObject randomPedestrianPrefab = pedestrianPrefabs[0];

            // Genera una posici�n aleatoria dentro del radio de spawn
            Vector3 randomSpawnPosition = RandomNavMeshPosition(transform.position, spawnRadius);

            // Instancia el prefab de peat�n en la posici�n aleatoria
            GameObject newPedestrian = Instantiate(randomPedestrianPrefab, randomSpawnPosition, Quaternion.identity);
        }
    }
        void SpawnHooligans()
    {
        for (int i = 0; i < numHooligans; i++)
        {
            // Selecciona un prefab de peat�n aleatorio del array
            GameObject randomHooliganPrefab = pedestrianPrefabs[1];

            // Genera una posici�n aleatoria dentro del radio de spawn
            Vector3 randomSpawnPosition = RandomNavMeshPosition(transform.position, spawnRadius);

            // Instancia el prefab de peat�n en la posici�n aleatoria
            GameObject newHooligan = Instantiate(randomHooliganPrefab, randomSpawnPosition, Quaternion.identity);
        }
    }

    Vector3 RandomNavMeshPosition(Vector3 center, float radius)
    {
        Vector3 randomDirection = Random.insideUnitSphere * radius;
        randomDirection += center;

        NavMeshHit hit;
        NavMesh.SamplePosition(randomDirection, out hit, radius, NavMesh.AllAreas);

        return hit.position;
    }
}
