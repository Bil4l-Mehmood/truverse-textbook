/**
 * User profile view and edit component
 */

import React, { useState, useEffect } from 'react';
import { useAuthStore } from '../../store/authStore';
import { updateProfile } from '../../services/authService';

export default function ProfileView() {
  const { user, token, isAuthenticated, updateUser, logout } = useAuthStore();

  const [isEditing, setIsEditing] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);

  // Edit form state
  const [ros2Experience, setRos2Experience] = useState(user?.ros2_experience || 'Beginner');
  const [gpuModel, setGpuModel] = useState(user?.gpu_model || '');
  const [gpuVram, setGpuVram] = useState(user?.gpu_vram || '');
  const [operatingSystem, setOperatingSystem] = useState(user?.operating_system || '');
  const [roboticsKnowledge, setRoboticsKnowledge] = useState(user?.robotics_knowledge || 'Beginner');

  // Redirect if not authenticated
  useEffect(() => {
    if (!isAuthenticated || !user || !token) {
      window.location.href = '/signin';
    }
  }, [isAuthenticated, user, token]);

  if (!isAuthenticated || !user || !token) {
    return null;
  }

  const handleSave = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setSuccess(false);
    setLoading(true);

    try {
      const updatedUser = await updateProfile(token, {
        ros2_experience: ros2Experience,
        gpu_model: gpuModel || undefined,
        gpu_vram: gpuVram || undefined,
        operating_system: operatingSystem || undefined,
        robotics_knowledge: roboticsKnowledge,
      });

      // Update local state
      updateUser(updatedUser);
      setIsEditing(false);
      setSuccess(true);

      // Hide success message after 3 seconds
      setTimeout(() => setSuccess(false), 3000);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to update profile');
    } finally {
      setLoading(false);
    }
  };

  const handleCancel = () => {
    // Reset form to current user values
    setRos2Experience(user.ros2_experience);
    setGpuModel(user.gpu_model || '');
    setGpuVram(user.gpu_vram || '');
    setOperatingSystem(user.operating_system || '');
    setRoboticsKnowledge(user.robotics_knowledge);
    setIsEditing(false);
    setError(null);
  };

  const handleLogout = () => {
    logout();
    window.location.href = '/';
  };

  return (
    <div className="auth-container">
      <div className="auth-card profile-card">
        <h1>User Profile</h1>

        {error && <div className="auth-error">{error}</div>}
        {success && <div className="auth-success">Profile updated successfully!</div>}

        <div className="profile-section">
          <h2>Account Information</h2>
          <div className="profile-field">
            <label>Name</label>
            <p>{user.name}</p>
          </div>
          <div className="profile-field">
            <label>Email</label>
            <p>{user.email}</p>
          </div>
          <div className="profile-field">
            <label>Member Since</label>
            <p>{new Date(user.created_at).toLocaleDateString()}</p>
          </div>
        </div>

        <div className="profile-section">
          <h2>Background Information</h2>

          {!isEditing ? (
            <>
              <div className="profile-field">
                <label>ROS 2 Experience</label>
                <p>{user.ros2_experience}</p>
              </div>
              <div className="profile-field">
                <label>GPU Model</label>
                <p>{user.gpu_model || 'Not specified'}</p>
              </div>
              <div className="profile-field">
                <label>GPU VRAM</label>
                <p>{user.gpu_vram || 'Not specified'}</p>
              </div>
              <div className="profile-field">
                <label>Operating System</label>
                <p>{user.operating_system || 'Not specified'}</p>
              </div>
              <div className="profile-field">
                <label>Robotics Knowledge</label>
                <p>{user.robotics_knowledge}</p>
              </div>

              <button
                type="button"
                onClick={() => setIsEditing(true)}
                className="btn-primary"
              >
                Edit Background Info
              </button>
            </>
          ) : (
            <form onSubmit={handleSave} className="auth-form">
              <div className="form-group">
                <label htmlFor="ros2-experience">ROS 2 Experience Level</label>
                <select
                  id="ros2-experience"
                  value={ros2Experience}
                  onChange={(e) => setRos2Experience(e.target.value)}
                >
                  <option value="None">None</option>
                  <option value="Beginner">Beginner</option>
                  <option value="Intermediate">Intermediate</option>
                  <option value="Advanced">Advanced</option>
                </select>
              </div>

              <div className="form-group">
                <label htmlFor="gpu-model">GPU Model</label>
                <input
                  type="text"
                  id="gpu-model"
                  value={gpuModel}
                  onChange={(e) => setGpuModel(e.target.value)}
                  placeholder="e.g., NVIDIA RTX 3060"
                />
              </div>

              <div className="form-group">
                <label htmlFor="gpu-vram">GPU VRAM</label>
                <input
                  type="text"
                  id="gpu-vram"
                  value={gpuVram}
                  onChange={(e) => setGpuVram(e.target.value)}
                  placeholder="e.g., 12GB"
                />
              </div>

              <div className="form-group">
                <label htmlFor="os">Operating System</label>
                <select
                  id="os"
                  value={operatingSystem}
                  onChange={(e) => setOperatingSystem(e.target.value)}
                >
                  <option value="">Select...</option>
                  <option value="Ubuntu">Ubuntu</option>
                  <option value="Windows">Windows</option>
                  <option value="macOS">macOS</option>
                </select>
              </div>

              <div className="form-group">
                <label htmlFor="robotics-knowledge">Robotics Knowledge Level</label>
                <select
                  id="robotics-knowledge"
                  value={roboticsKnowledge}
                  onChange={(e) => setRoboticsKnowledge(e.target.value)}
                >
                  <option value="None">None</option>
                  <option value="Beginner">Beginner</option>
                  <option value="Intermediate">Intermediate</option>
                  <option value="Advanced">Advanced</option>
                </select>
              </div>

              <div className="form-actions">
                <button
                  type="button"
                  onClick={handleCancel}
                  className="btn-secondary"
                  disabled={loading}
                >
                  Cancel
                </button>
                <button type="submit" className="btn-primary" disabled={loading}>
                  {loading ? 'Saving...' : 'Save Changes'}
                </button>
              </div>
            </form>
          )}
        </div>

        <div className="profile-actions">
          <button onClick={handleLogout} className="btn-danger">
            Sign Out
          </button>
        </div>
      </div>
    </div>
  );
}
