/*
*/
#ifndef _GUI_VEHICLE_GUI_HH_
#define _GUI_VEHICLE_GUI_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

#include "robot.h"

#define DEG2RAD(x)  x*0.017453293
#define RAD2DEG(x)  x*57.29577951

namespace gazebo
{
    class GAZEBO_VISIBLE VehicleGUI : public GUIPlugin
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget
      public: VehicleGUI();

      /// \brief Destructor
      public: virtual ~VehicleGUI();


		/// \brief Callback trigged when the button is pressed.
		protected slots: void OnButton();

		private slots: void OnAutoDrive();
    private slots: void OnIncreaseGas();
    private slots: void OnDecreaseGas();
    private slots: void OnStop();
    private slots: void OnTurnLeft();
    private slots: void OnTurnRight();
    private slots: void OnGoStraight();
    private slots: void OnSlowDrive();
    private slots: void OnNormalDrive();
    private slots: void OnFastDrive();
    private slots: void Send();
    /// \brief SDF for this plugin.
    private: sdf::ElementPtr sdf;

		private: double
			velocity_inc = 0.1,
			velocity = 0.0,
			angle_max = 90.0,
			angle_inc = 1.0,
			angle = 0.0;

      /// \brief Counter used to create unique model names
      private: unsigned int counter;

      /// \brief Node used to establish communication with gzserver.
			private: transport::NodePtr camera;
    	private: transport::NodePtr drive;

      /// \brief Publisher of factory messages.
//?      private: transport::PublisherPtr factoryPub;
			/// \brief Control publisher.
			private: transport::PublisherPtr controlPub;
    };
}
#endif
