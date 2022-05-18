/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once  // NOLINT(build/header_guard)

#include <ros/ros.h>
#include <QVBoxLayout>
#include <QPushButton>
#include <QRadioButton>
#include <QGroupBox>
#include <QComboBox>
#include <QDialog>
#include <QTabWidget>
#include <QLabel>
#include <QLineEdit>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <string>
#include <vector>

namespace rosdyn_gui
{


class GenerationTab : public QWidget
{
  Q_OBJECT

public:
  explicit GenerationTab(ros::NodeHandle& nh, QWidget *parent = 0);

  QGroupBox* m_formGroupBox;
  QGridLayout* m_grid_layout;
  ros::NodeHandle m_nh;

protected Q_SLOTS:
  void changedDuration(double value);
  void changedStage2(int number);
  void changedPointStage2(int number);
  void changeTrialNumber(int number);
  void saveNewPar();
  void changeFrequency(double value);
  void changeSamplingPeriod(double value);

protected:
  QPushButton* m_ok_btn;
  QPushButton* m_cancel_btn;
  double m_stage_duration;
  int m_regione_stage2;
  int m_point_per_region;
  int m_trials;
  double m_frequency;
  double m_sampling_period;
};

class TabDialog : public QDialog
{
  Q_OBJECT

public:
  explicit TabDialog(ros::NodeHandle& nh, QWidget *parent = 0);

private:
  QTabWidget* m_tabWidget;
  ros::NodeHandle m_nh;
};


}  // namespace rosdyn_gui
