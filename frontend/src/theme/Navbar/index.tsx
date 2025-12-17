/**
 * Custom Navbar with authentication links
 */

import React from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import { useAuthStore } from '../../store/authStore';
import './styles.css';

export default function NavbarWrapper(props) {
  return (
    <>
      <OriginalNavbar {...props} />
    </>
  );
}
