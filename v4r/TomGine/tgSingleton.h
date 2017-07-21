 /**
 * @file tgSingleton
 * @author Thomas Mörwald
 * @date August 2011
 * @version 0.1
 */
 
#ifndef _TG_SINGLETON_H_
#define _TG_SINGLETON_H_

namespace TomGine{

/** @brief singleton class for global instantiation. */
template <typename T>
class tgSingleton
{
public:
	static T* GetInstance(){
		if (!m_instance)
			m_instance = new T ();
		return m_instance;
	}
	virtual ~tgSingleton(){
	  if (m_instance)
	    delete(m_instance);
		m_instance = 0;
	}
private:
	static T* m_instance;
protected:
	tgSingleton() { }
};

template <typename T>
T* tgSingleton <T>::m_instance = 0;

}

#endif
