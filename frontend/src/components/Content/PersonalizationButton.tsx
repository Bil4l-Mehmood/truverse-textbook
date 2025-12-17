/**
 * Personalization Button Component
 * Displays user preferences and allows customizing content based on:
 * - ROS 2 Experience Level (Beginner/Intermediate/Advanced)
 * - Robotics Knowledge Level (None/Beginner/Intermediate/Advanced)
 * - GPU availability
 * - Operating System
 */

import React, { useState } from 'react';
import { useAuthStore } from '../../store/authStore';
import './personalization-button.css';

interface PersonalizationButtonProps {
  chapterId?: string;
  onPersonalize?: (preferences: PersonalizationPreferences) => void;
}

export interface PersonalizationPreferences {
  ros2Level: string;
  roboticsLevel: string;
  showGPUOptimizations: boolean;
  operatingSystem: string;
}

export default function PersonalizationButton({
  chapterId,
  onPersonalize,
}: PersonalizationButtonProps) {
  const user = useAuthStore((state) => state.user);
  const [showModal, setShowModal] = useState(false);
  const [preferences, setPreferences] = useState<PersonalizationPreferences>({
    ros2Level: user?.ros2_experience || 'Beginner',
    roboticsLevel: user?.robotics_knowledge || 'Beginner',
    showGPUOptimizations: !!user?.gpu_model,
    operatingSystem: user?.operating_system || 'Ubuntu',
  });

  const handleApply = async () => {
    console.log('[Personalization] Applying preferences:', preferences);

    try {
      // Save to backend if user is authenticated
      const token = localStorage.getItem('auth_token');
      if (token) {
        console.log('[Personalization] Saving preferences to backend...');
        const response = await fetch('http://localhost:8000/api/v1/auth/profile', {
          method: 'PUT',
          headers: {
            'Content-Type': 'application/json',
            'Authorization': `Bearer ${token}`,
          },
          body: JSON.stringify({
            ros2_experience: preferences.ros2Level,
            robotics_knowledge: preferences.roboticsLevel,
            operating_system: preferences.operatingSystem,
            gpu_model: user?.gpu_model || undefined,
          }),
        });

        if (!response.ok) {
          console.warn('[Personalization] Failed to save to backend:', response.statusText);
        } else {
          console.log('[Personalization] Preferences saved to backend');
        }
      }
    } catch (err) {
      console.error('[Personalization] Error saving preferences:', err);
    }

    if (onPersonalize) {
      onPersonalize(preferences);
    }
    setShowModal(false);

    // Store preference in localStorage for persistence
    localStorage.setItem(`personalization-${chapterId}`, JSON.stringify(preferences));
  };

  return (
    <>
      <button
        className="personalization-btn"
        onClick={() => setShowModal(true)}
        title="Personalize this chapter based on your preferences"
      >
        🎯 Personalize Content
      </button>

      {showModal && (
        <div className="personalization-modal-overlay" onClick={() => setShowModal(false)}>
          <div
            className="personalization-modal"
            onClick={(e) => e.stopPropagation()}
          >
            <div className="personalization-header">
              <h2>📚 Personalize Your Learning</h2>
              <button
                className="close-btn"
                onClick={() => setShowModal(false)}
              >
                ✕
              </button>
            </div>

            <div className="personalization-content">
              <p className="intro-text">
                Customize this chapter's content based on your experience level and setup.
              </p>

              <div className="preference-group">
                <label htmlFor="ros2-level">
                  <strong>ROS 2 Experience Level</strong>
                </label>
                <select
                  id="ros2-level"
                  value={preferences.ros2Level}
                  onChange={(e) =>
                    setPreferences({ ...preferences, ros2Level: e.target.value })
                  }
                >
                  <option value="None">None - Show all fundamentals</option>
                  <option value="Beginner">Beginner - Basic concepts</option>
                  <option value="Intermediate">Intermediate - Advanced topics</option>
                  <option value="Advanced">Advanced - Deep dives</option>
                </select>
              </div>

              <div className="preference-group">
                <label htmlFor="robotics-level">
                  <strong>Robotics Knowledge</strong>
                </label>
                <select
                  id="robotics-level"
                  value={preferences.roboticsLevel}
                  onChange={(e) =>
                    setPreferences({ ...preferences, roboticsLevel: e.target.value })
                  }
                >
                  <option value="None">None - Show hardware basics</option>
                  <option value="Beginner">Beginner - Basic robotics</option>
                  <option value="Intermediate">Intermediate - Robot control</option>
                  <option value="Advanced">Advanced - Complex systems</option>
                </select>
              </div>

              <div className="preference-group">
                <label htmlFor="os">
                  <strong>Operating System</strong>
                </label>
                <select
                  id="os"
                  value={preferences.operatingSystem}
                  onChange={(e) =>
                    setPreferences({ ...preferences, operatingSystem: e.target.value })
                  }
                >
                  <option value="Ubuntu">Ubuntu / Linux</option>
                  <option value="Windows">Windows</option>
                  <option value="macOS">macOS</option>
                </select>
              </div>

              <div className="preference-group checkbox">
                <label>
                  <input
                    type="checkbox"
                    checked={preferences.showGPUOptimizations}
                    onChange={(e) =>
                      setPreferences({
                        ...preferences,
                        showGPUOptimizations: e.target.checked,
                      })
                    }
                  />
                  <span>Show GPU optimization examples (GPU: {user?.gpu_model || 'Not specified'})</span>
                </label>
              </div>

              <div className="preference-info">
                <p>✨ <strong>Personalized content will:</strong></p>
                <ul>
                  <li>Skip basic concepts if you're advanced</li>
                  <li>Show OS-specific installation commands</li>
                  <li>Include GPU acceleration examples if available</li>
                  <li>Match code examples to your experience level</li>
                </ul>
              </div>
            </div>

            <div className="personalization-footer">
              <button
                className="btn-secondary"
                onClick={() => setShowModal(false)}
              >
                Cancel
              </button>
              <button className="btn-primary" onClick={handleApply}>
                ✨ Apply Personalization
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
}
