 /**
 * @file tgTimer.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Real-time clock for MS Windows and Linux.
 */

#ifndef TG_TIMER
#define TG_TIMER

#ifdef _WIN32
	#include <windows.h>
#else
	#include <time.h>
	#include <sys/time.h>
#endif

// ***********************************************************************************

namespace TomGine{

/** @brief Timer handling frame- and application times. */
class tgTimer
{
private:
#ifdef WIN32
	LONGLONG m_StartTicks;		// QueryPerformance - Ticks at application start
	LONGLONG m_EndTicks;		// QueryPerformance - Ticks when calling Now()
	LONGLONG m_Frequency;		// QueryPerformance - Fequency
	double fNow;
#else	
	struct timespec AppStart, act, old;
#endif
	double m_fAppTime;			// Time since application started
	double m_fTime;				// Time between two Update calls

public:
	/** @brief Create and start timer. */
	tgTimer(void);
	
	/** @brief Reset application time and start timer. */
	void	Reset();
	/** @brief Calculate time since last call to Update and update application time.
	 *  @return Time since last call of Update(). */
	double	Update();
	
	/** @brief Get Time since last call to Update. Does not update application time.
	 *  @return Time since last call of Update(). */
	double	GetFrameTime() const { return m_fTime;}

	/** @brief Get Time since last call of Reset() or constructor tgTimer().
	 *  @return Time since last call of Reset() or constructor tgTimer(). */
	double	GetApplicationTime() const { return m_fAppTime;}
};

} // namespace TomGine

#endif

