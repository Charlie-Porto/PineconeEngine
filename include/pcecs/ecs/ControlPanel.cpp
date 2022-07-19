#ifndef ControlPanel_cpp
#define ControlPanel_cpp

#include <memory>

#include "EntityMgr.cpp"
#include "ComponentMgr.cpp"
#include "SystemMgr.cpp"


class ControlPanel
{
public:
	void Init()
	{
			mEntityManager = std::make_unique<EntityMgr>();
			mComponentManager = std::make_unique<ComponentMgr>();
			mSystemManager = std::make_unique<SystemMgr>();
	}
    // Entity methods
	Entity CreateEntity()
	{
		return mEntityManager->CreateEntity();
	}

	void DestroyEntity(Entity entity)
	{
		mEntityManager->DestroyEntity(entity);

		mComponentManager->EntityDestroyed(entity);

		mSystemManager->EntityDestroyed(entity);
	}


	// Component methods
	template<typename T>
	void RegisterComponent()
	{
		mComponentManager->RegisterComponent<T>();
	}

	template<typename T>
	void AddComponent(Entity entity, T component)
	{
		mComponentManager->AddComponent<T>(entity, component);

		auto signature = mEntityManager->GetSignature(entity);
		signature.set(mComponentManager->GetComponentType<T>(), true);
		mEntityManager->SetSignature(entity, signature);

		mSystemManager->EntitySignatureChanged(entity, signature);
	}

	template<typename T>
	void RemoveComponent(Entity entity)
	{
		mComponentManager->RemoveComponent<T>(entity);

		auto signature = mEntityManager->GetSignature(entity);
		signature.set(mComponentManager->GetComponentType<T>(), false);
		mEntityManager->SetSignature(entity, signature);

		mSystemManager->EntitySignatureChanged(entity, signature);
	}

	template<typename T>
	T& GetComponent(Entity entity)
	{
		return mComponentManager->GetComponent<T>(entity);
	}

	template<typename T>
	int GetComponentType()
	{
		return mComponentManager->GetComponentType<T>();
	}


	// System methods
	template<typename T>
	std::shared_ptr<T> RegisterSystem()
	{
		return mSystemManager->RegisterSystem<T>();
	}

	template<typename T>
	void SetSystemSignature(Signature signature)
	{
		mSystemManager->SetSignature<T>(signature);
	}
  
	// System methods for succint registration
	template<typename T0, typename T1>
	void AssignSystemComponents()
  {
		Signature sig;
		sig.set(GetComponentType<T1>());
		SetSystemSignature<T0>(sig);
	}

	template<typename T0, typename T1, typename T2>
	void AssignSystemComponents()
  {
		Signature sig;
		sig.set(GetComponentType<T1>());
		sig.set(GetComponentType<T2>());
		SetSystemSignature<T0>(sig);
	}

	template<typename T0, typename T1, typename T2, typename T3>
	void AssignSystemComponents()
  {
		Signature sig;
		sig.set(GetComponentType<T1>());
		sig.set(GetComponentType<T2>());
		sig.set(GetComponentType<T3>());
		SetSystemSignature<T0>(sig);
	}

	template<typename T0, typename T1, typename T2, typename T3, typename T4>
	void AssignSystemComponents()
  {
		Signature sig;
		sig.set(GetComponentType<T1>());
		sig.set(GetComponentType<T2>());
		sig.set(GetComponentType<T3>());
		sig.set(GetComponentType<T4>());
		SetSystemSignature<T0>(sig);
	}

private:
    std::unique_ptr<EntityMgr> mEntityManager;
    std::unique_ptr<ComponentMgr> mComponentManager;
    std::unique_ptr<SystemMgr> mSystemManager;
};




#endif /* ControlPanel_cpp */
